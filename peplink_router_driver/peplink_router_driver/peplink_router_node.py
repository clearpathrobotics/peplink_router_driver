#!/usr/bin/env python3

# Copyright 2025 Clearpath Robotics Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Clearpath Robotics Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from collections import OrderedDict
import math
import os
import subprocess
import sys

from diagnostic_msgs.msg import (
    DiagnosticStatus
)

from diagnostic_updater import (
    DiagnosticStatusWrapper,
    Updater,
)

from peplink_msgs.msg import (
    Band,
    Bandwidth,
    Client,
    ClientList,
    Firmware,
    Lan,
    LanList,
    Lease,
    Rat,
    Sim,
    Wan,
    WanList,
    WanPriority,
)

from peplink_msgs.srv import (
    GetFirmware,
    SetWanPriority,
)

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix

import urllib3

from wireless_msgs.msg import Connection

from peplink_router_driver.checker import Authentication, PeplinkCheck
from peplink_router_driver.periodic import PeriodicCheck


class PeplinkRouterNode(Node):
    """
    The main interface class for the Peplink router.

    Provides basic access to the router state, allows users to change between wifi and cellular
    modes.
    """

    @property
    def ip_address(self):
        return self.ip_address_param.value

    @property
    def username(self):
        return self.username_param.value

    @property
    def password(self):
        return self.password_param.value

    @property
    def http_headers(self):
        return {
            'User-Agent': (
                'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko)'
                ' Chrome/111.0.0.0 Safari/537.36'
            ),
            'Content-Type': 'application/json',
        }

    @property
    def publish_passwords(self):
        return self.publish_passwords_param.value

    def __init__(self, node_name: str):
        # We don't check the HTTPS certificate, as it's not always valid for the router's subnet
        # but this causes unnecessary warnings in the logs. To keep things tidy, turn off
        # these warnings.
        # See: https://urllib3.readthedocs.io/en/latest/advanced-usage.html#tls-warnings
        urllib3.disable_warnings()

        super().__init__(node_name=node_name, cli_args=sys.argv)

        self.diagnostics = OrderedDict()

        self.updater = Updater(self, 1.0)
        self.updater.setHardwareID(f'peplink ({self.get_name()})')

        self.updater.add(
            self.get_name().replace('_node', '').replace('_', ' ').title(),
            self.generate_diagnostics,
        )

        # ROS parameter declarations
        self.ip_address_param = self.declare_parameter(
            'ip_address',
            '192.168.131.51',
        )
        self.username_param = self.declare_parameter(
            'username',
            'admin',
        )
        self.password_param = self.declare_parameter(
            'password',
            'admin',
        )
        self.enable_gps_param = self.declare_parameter(
            'enable_gps',
            False,
        )
        self.publish_passwords_param = self.declare_parameter(
            'publish_passwords',
            False,
        )

        # web connection management
        self.authentication = Authentication(
            self,
            self.ip_address,
            self.username,
            self.password
        )
        self.wait_for_host()
        self.diagnostics['Authenticated'] = False
        self.diagnostics['Authenticated'] = self.authentication.login()

        # Service servers
        self.get_firmware_srv = self.create_service(
            GetFirmware,
            'get_firmware',
            self.get_firmware_handler,
        )
        self.set_wan_priority_srv = self.create_service(
            SetWanPriority,
            'set_wan_priority',
            self.set_wan_priority_handler,
        )

        # Topic publishers
        self.clients_pub = self.create_publisher(
            ClientList,
            'clients',
            qos_profile=qos_profile_sensor_data,
        )
        self.lans_pub = self.create_publisher(
            LanList,
            'lans',
            qos_profile=qos_profile_sensor_data,
        )
        self.wans_pub = self.create_publisher(
            WanList,
            'wans',
            qos_profile=qos_profile_sensor_data,
        )
        self.wifi_24g_signal_pub = self.create_publisher(
            Connection,
            'connection/wifi_24g',
            qos_profile=qos_profile_sensor_data,
        )
        self.wifi_5g_signal_pub = self.create_publisher(
            Connection,
            'connection/wifi_5g',
            qos_profile=qos_profile_sensor_data,
        )
        self.cellular_signal_pub = self.create_publisher(
            Connection,
            'connection/cellular',
            qos_profile=qos_profile_sensor_data,
        )

        # Periodic checkers
        self.periodic_checks = [
            PeriodicCheck(
                self,
                f'https://{self.ip_address}/api/status.client?connectionType=ethernet wireless',
                1.0,
                self.publish_clients,
                self.authentication,
            ),
            PeriodicCheck(
                self,
                f'https://{self.ip_address}/api/status.lan.profile',
                1.0,
                self.publish_lans,
                self.authentication,
            ),
            PeriodicCheck(
                self,
                f'https://{self.ip_address}/api/status.wan.connection',
                1.0,
                self.publish_wans,
                self.authentication,
            ),
        ]

        # GPS publisher is optional
        if self.enable_gps_param.value:
            self.navsat_fix_pub = self.create_publisher(
                NavSatFix,
                'gps_0/fix',
                qos_profile=qos_profile_sensor_data,
            )
            self.periodic_checks.append(
                PeriodicCheck(
                    self,
                    f'https://{self.ip_address}/api/info.location',
                    1.0,
                    self.publish_gps,
                )
            )

    def generate_diagnostics(self, stat: DiagnosticStatusWrapper):
        stat.summary(DiagnosticStatus.OK, 'OK')

        if not self.diagnostics.get('Alive', False):
            stat.mergeSummary(DiagnosticStatus.WARN, f'Unable to ping router at {self.ip_address}')

        if not self.diagnostics.get('Authenticated', False):
            stat.mergeSummary(DiagnosticStatus.WARN, 'Not authenticated')

        for key in self.diagnostics.keys():
            stat.add(key, str(self.diagnostics[key]))

        return stat

    def wait_for_host(self):
        """
        Wait for host to come up.

        Wait until the host is actually online before we try to contact it.
        This reduces http related errors
        """
        self.diagnostics['Alive'] = False

        # ping syntax is different on Windows than Linux, so set the command accordingly
        if os.name == 'nt':
            cmd = f'ping -W 5 -n 1 {self.ip_address}'.split()
        else:
            cmd = f'ping -W 5 -c 1 {self.ip_address}'.split()

        self.get_logger().info(f'Waiting until {self.ip_address} is online...')
        host_alive = subprocess.call(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ) == 0
        rate = self.create_rate(1)
        while not host_alive:
            rate.sleep()
            host_alive = subprocess.call(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ) == 0

        self.get_logger().info(f'{self.ip_address} is now online')
        self.diagnostics['Alive'] = True

    def get_firmware_handler(self, request, result):
        checker = PeplinkCheck(
            self,
            f'https://{self.ip_address}/api/info.firmware',
            self.authentication,
        )
        data = checker.get_json()
        order = data.get('response', {}).get('order', [])
        for n in order:
            fw_json = data.get('response', {}).get(f'{n}', {})
            fw = Firmware()
            fw.id = n
            fw.version = fw_json.get('version', '')
            fw.bootable = fw_json.get('bootable', False)
            fw.in_use = fw_json.get('inUse', False)
            result.firmwares.append(fw)

        result.stat.code = data.get('code', 200)
        result.stat.stat = data.get('stat', '')
        result.stat.message = data.get('message', '')

        return result

    def set_wan_priority_handler(self, request, result):
        checker = PeplinkCheck(
            self,
            f'https://{self.ip_address}/api/config.wan.priority'
        )
        content = {
            'instantActive': request.instant_actve,
            'list': []
        }
        for wan in request.connections:
            conn = {
                'connId': wan.id,
                'priority': wan.priority,
                'group': wan.group,
                'enable': wan.enable
            }
            content['list'].append(conn)

        data = checker.post_json(content)
        order = data.get('response', {}).get('order', [])
        for n in order:
            wan_json = data.get('response', {}).get(f'{n}', {})
            wan = WanPriority()
            wan.name = wan_json.get('name', '')
            wan.id = n
            wan.group = wan_json.get('group', 0)
            wan.enable = wan_json.get('enable', False)

            result.connections.append(wan)

        result.stat.code = data.get('code', 200)
        result.stat.stat = data.get('stat', '')
        result.stat.message = data.get('message', '')

        return result

    def publish_clients(self, data):
        clients = ClientList()
        clients.stat.code = data.get('code', 200)
        clients.stat.stat = data.get('stat', '')
        clients.stat.message = data.get('message', '')

        client_list = data.get('response', {}).get('list', [])

        active_clients = 0
        client_ips = []

        for client_json in client_list:
            client = Client()
            client.ip_address = client_json.get('ip', '')
            client.connection_type = client_json.get('connectionType', 'other')
            client.name = client_json.get('name', '')
            client.mac = client_json.get('mac', '')
            client.active = client_json.get('active', False)
            client.vlan_id = client_json.get('vlanId', -1)

            client.bandwidth = Bandwidth()
            client.bandwidth.download = client_json.get('speed', {}).get('download', 0)
            client.bandwidth.upload = client_json.get('speed', {}).get('upload', 0)
            client.bandwidth.unit = client_json.get('speed', {}).get('unit', '')

            client.wifi_connection = Connection()
            client.wifi_connection.essid = client_json.get('essid', '')
            client.wifi_connection.bssid = client_json.get('bssid', '')
            client.wifi_connection.txpower = client_json.get('signalStrength', {}).get('value', 0)
            client.wifi_connection.signal_level = client_json.get('signal', {}).get('strength', 0)
            client.wifi_connection.bitrate = (client.bandwidth.download + client.bandwidth.upload) / 2.0  # noqa: E501
            client.wifi_connection.link_quality = client_json.get('signal', {}).get('level', 0) / 5.0  # noqa: E501
            client.wifi_connection.link_quality_raw = str(client_json.get('signal', {}).get('level', 0))  # noqa: E501

            client.lease = Lease()
            client.lease.expires_in = client_json.get('lease', {}).get('expiresIn', 0)
            client.lease.type = client_json.get('lease', {}).get('type', '')

            clients.clients.append(client)

            if client.active:
                active_clients += 1
                client_ips.append(client.ip_address)

        self.clients_pub.publish(clients)

        self.diagnostics['Clients'] = active_clients
        self.diagnostics['Client IP addresses'] = client_ips

    def publish_lans(self, data):
        lans = LanList()
        lans.stat.code = data.get('code', 200)
        lans.stat.stat = data.get('stat', '')
        lans.stat.message = data.get('message', '')

        order = data.get('response', {}).get('order', [])

        active_lans = 0
        lan_ips = []

        for n in order:
            lan_json = data.get('response', {}).get(f'{n}', {})
            lan = Lan()
            lan.id = n
            lan.name = lan_json.get('name', '')
            lan.vlan_id = lan_json.get('vlanId', 0)
            lan.ip_address = lan_json.get('ip', '')
            lan.netmask = lan_json.get('mask', 0)
            lans.lans.append(lan)

            if lan.ip_address:
                active_lans += 1
                if lan.ip_address:
                    lan_ips.append(lan.ip_address)

        self.lans_pub.publish(lans)

        self.diagnostics['LANs'] = active_lans
        self.diagnostics['LAN IP addresses'] = lan_ips

    def publish_wans(self, data):
        def parse_bands(band_arr, dest):
            for band_json in band_arr:
                band = Band()
                band.name = band_json.get('name', '')
                band.channel = band_json.get('channel', 0)

                signal_json = band_json.get('signal', {})
                signal = band.signal
                signal.rssi = signal_json.get('rssi', 0)
                signal.sinr = signal_json.get('sinr', 0.0)
                signal.snr = signal_json.get('snr', 0.0)
                signal.ecio = signal_json.get('ecio', 0.0)
                signal.rsrp = signal_json.get('rsrp', 0)
                signal.rsrq = signal_json.get('rsrq', 0.0)
                signal.strength = signal_json.get('strength', 0)

                dest.append(band)

        def parse_gobi(cell_json, cell):
            cell.network = cell_json.get('network', '')
            cell.mobile_type = cell_json.get('mobileType', '')
            cell.carrier_aggregation = cell_json.get('carrierAggregation', False)
            cell.signal_level = cell_json.get('signalLevel', 0)
            cell.imei = cell_json.get('imei', '')
            cell.esn = cell_json.get('esn', '')
            cell.eid = cell_json.get('eid', '')
            cell.network_mode = cell_json.get('network_mode', '')
            cell.manufacturer = cell_json.get('manufacturer', '')
            cell.model = cell_json.get('model', '')
            cell.firmware = cell_json.get('firmware', '')
            cell.mode = cell_json.get('mode', '')
            cell.mcc = cell_json.get('mcc', '')
            cell.mnc = cell_json.get('mnc', '')

            parse_bands(cell_json.get('band', []), cell.band)

            for rat_json in cell_json.get('rat', []):
                rat = Rat()
                rat.name = rat_json.get('name', '')
                parse_bands(rat_json.get('band', []), rat.band)
                cell.rat.append(rat)

            roam_json = cell_json.get('roaming', {})
            roam = cell.roaming
            roam.code = roam_json.get('code', 0)
            roam.message = roam_json.get('message', '')

            sim_group_json = cell_json.get('sim', {})
            order = sim_group_json.get('order', [])
            for n in order:
                sim_json = sim_group_json.get(f'{n}', {})
                sim = Sim()
                sim.id = n
                sim.status = sim_json.get('status', '')
                sim.active = sim_json.get('active', False)
                sim.apn = sim_json.get('apn', '')
                sim.username = sim_json.get('username', '')
                if self.publish_passwords:
                    sim.password = sim_json.get('password', '')
                else:
                    sim.password = ''
                sim.imsi = sim_json.get('imsi', '')
                sim.iccid = sim_json.get('iccid', '')
                sim.mtn = sim_json.get('mtn', '')

                cell.sim.append(sim)

            rsim_json = cell_json.get('remoteSim', {})
            rsim = cell.remote_sim
            rsim.imsi = rsim_json.get('imsi', '')
            rsim.serial_no = rsim_json.get('serial_no', '')
            rsim.slot = rsim_json.get('slot', 0)
            rsim.auto_apn = rsim_json.get('autoApn', False)
            rsim.apn = rsim_json.get('apn', '')
            rsim.username = rsim_json.get('username', '')
            if self.publish_passwords:
                rsim.password = rsim_json.get('password', '')
            else:
                rsim.password = ''

            carrier_json = cell_json.get('carrier', {})
            carrier = cell.carrier
            carrier.name = carrier_json.get('name', '')
            carrier.country = carrier_json.get('country', '')

            meid_json = cell_json.get('meid', {})
            meid = cell.meid
            meid.hex = meid_json.get('hex', '')
            meid.dec = meid_json.get('dec', '')

            tower_json = cell_json.get('cellTower', {})
            tower = cell.cell_tower
            tower.id = tower_json.get('cellId', 0)
            tower.plmn = tower_json.get('cellPlmn', 0)
            tower.utranid = tower_json.get('cellUtranId', 0)
            tower.tac = tower_json.get('tac', 0)
            tower.lac = tower_json.get('lac', 0)

        # import json
        # self.get_logger().info(json.dumps(data))

        wans = WanList()
        wans.stat.code = data.get('code', 200)
        wans.stat.stat = data.get('stat', '')
        wans.stat.message = data.get('message', '')

        active_wans = 0
        primary_wan_ip = ''
        wan_ips = []

        order = data.get('response', {}).get('order', [])
        for n in order:
            wan_json = data.get('response', {}).get(f'{n}', {})
            wan = Wan()
            wan.id = n
            wan.name = wan_json.get('name', '')
            wan.led_color = wan_json.get('statusLed', '')
            wan.as_lan = wan_json.get('asLan', False)
            wan.enable = wan_json.get('enable', False)
            wan.locked = wan_json.get('locked', False)
            wan.scheduled_off = wan_json.get('scheduledOff', False)
            wan.message = wan_json.get('message', '')
            wan.uptime = wan_json.get('uptime', -1)
            wan.type = wan_json.get('type', '')
            wan.virtual_type = wan_json.get('virtualType', '')
            wan.priority = wan_json.get('priority', 0)
            wan.group_set = wan_json.get('groupSet', 0)
            wan.ip_address = wan_json.get('ip', '')
            wan.netmask = wan_json.get('mask', 0)
            wan.gateway = wan_json.get('gateway', '')
            wan.method = wan_json.get('method', '')
            wan.mode = wan_json.get('mode', '')
            wan.routing_mode = wan_json.get('routingMode', '')
            for ip in wan_json.get('dns', []):
                wan.dns.append(ip)
            for ip in wan_json.get('additionalIp', []):
                wan.additional_ip.append(ip)
            wan.mtu = wan_json.get('mtu', 0)
            wan.mss = wan_json.get('mss', 0)

            allowance_json = wan_json.get('bandwidthAllowanceMonitor', {})
            allowance = wan.allowance
            allowance.enable = allowance_json.get('enable', False)
            allowance.has_smtp = allowance_json.get('hasSmtp', False)
            for action in allowance_json.get('action', []):
                allowance_json.action.append(action)
            allowance.start = allowance_json.get('start', 0)
            allowance.value = allowance_json.get('monthlyAllowance', {}).get('value', 0.0)
            allowance.unit = allowance_json.get('monthlyAllowance', {}).get('unit', 'MB')

            # WAN over Wifi
            wifi_json = wan_json.get('wifi', {})
            signal_json = wifi_json.get('signal', {})
            wifi = wan.wireless
            wifi.essid = wifi_json.get('ssid', '')
            wifi.bssid = wifi_json.get('bssid', '')
            wifi.signal_level = signal_json.get('strength', 0)  # dBm
            wifi.link_quality_raw = str(signal_json.get('level', ''))
            wifi.link_quality = float(signal_json.get('level', 0))
            if '2.4 GHz' in wan.name:
                wifi.frequency = 2.4
            elif '5 GHz' in wan.name:
                wifi.frequency = 5.0

            # WAN over Modem
            modem_json = wan_json.get('modem', {})
            modem = wan.modem
            modem.name = modem_json.get('name', '')
            modem.vendor_id = modem_json.get('vendorId', 0)
            modem.product_id = modem_json.get('productId', 0)
            modem.manufacturer = modem_json.get('manufacturer', '')
            modem.signal_level = modem_json.get('signalLevel', 0)
            modem.network = modem_json.get('network', '')
            modem.imsi = modem_json.get('imsi', '')
            modem.iccid = modem_json.get('iccid', '')
            modem.esn = modem_json.get('esn', '')
            modem.mtn = modem_json.get('mtn', '')
            modem.apn = modem_json.get('apn', '')
            modem.username = modem_json.get('username', '')
            if self.publish_passwords:
                modem.password = modem_json.get('password', '')
            else:
                modem.password = ''
            modem.dial_number = modem_json.get('dialNumber', '')
            carrier_json = modem_json.get('carrier', {})
            carrier = modem.carrier
            carrier.name = carrier_json.get('name', '')
            carrier.country = carrier_json.get('country', '')
            parse_bands(modem_json.get('band', []), modem.band)

            # WAN over cellular
            parse_gobi(wan_json.get('cellular', {}), wan.cellular)

            # WAN over gobi
            parse_gobi(wan_json.get('gobi', {}), wan.gobi)

            # add the WAN connection to the list
            wans.wans.append(wan)

            # check for specific connections that get their own topics
            if wan.type == 'wifi':
                if '2.4 GHz' in wan.name and 'Connected' in wan.message:
                    self.wifi_24g_signal_pub.publish(wan.wireless)
                elif '5 GHz' in wan.name and 'Connected' in wan.message:
                    self.wifi_5g_signal_pub.publish(wan.wireless)
            elif wan.type == 'cellular':
                if 'Connected' in wan.message:
                    self.cellular_signal_pub.publish(wan.wireless)

            if wan.enable:
                active_wans += 1
                if not primary_wan_ip:
                    primary_wan_ip = wan.ip_address
                if wan.ip_address:
                    wan_ips.append(wan.ip_address)

        self.wans_pub.publish(wans)
        self.diagnostics['WANs'] = active_wans
        self.diagnostics['WAN Primary IP'] = primary_wan_ip
        self.diagnostics['WAN IP addresses'] = wan_ips

    def publish_gps(self, data):
        fix = NavSatFix()
        if data['stat'] == 'fail':
            self.get_logger().warn('Failed to read GPS data')
            fix.latitude = math.nan
            fix.longitude = math.nan
            fix.altitude = math.nan
        elif not data.get('response', {}).get('gps', False):
            self.get_logger().warn('GPS data is invalid')
            fix.latitude = math.nan
            fix.longitude = math.nan
            fix.altitude = math.nan
        else:
            gps_json = data.get('response', {}).get('location', {})
            fix.latitude = gps_json.get('latitude', math.nan)
            fix.longitude = gps_json.get('longitude', math.nan)
            fix.altitude = gps_json.get('altitude', math.nan)
        fix.header.stamp = self.get_clock().now().to_msg()
        self.navsat_fix_pub.publish(fix)


def main():
    rclpy.init(args=sys.argv)

    node_name = 'peplink_router_node'
    executor = MultiThreadedExecutor()
    node = PeplinkRouterNode(node_name)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
