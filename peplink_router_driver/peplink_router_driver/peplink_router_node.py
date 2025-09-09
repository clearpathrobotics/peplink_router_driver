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

import json
import math
import os
import subprocess
import threading

from peplink_msgs.msg import (
    Bandwidth,
    Client,
    ClientList,
    Firmware,
    Lease,
    Signal,
    SignalDetails,
)
from peplink_msgs.srv import GetFirmware

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import requests

from sensor_msgs.msg import NavSatFix

import urllib3


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

    def __init__(self, node_name):
        # We don't check the HTTPS certificate, as it's not always valid for the router's subnet
        # but this causes unnecessary warnings in the logs. To keep things tidy, turn off
        # these warnings.
        # See: https://urllib3.readthedocs.io/en/latest/advanced-usage.html#tls-warnings
        urllib3.disable_warnings()

        super().__init__(node_name=node_name)

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

        # web connection management
        self.session = requests.Session()
        self.wait_for_host()
        n_tries = 0
        max_tries = 3
        logged_in = False
        while n_tries < max_tries and not logged_in:
            n_tries += 1
            logged_in = self.login()
        if not logged_in:
            self.get_logger().info(f'Login failed after {max_tries} attempts. Some features may be disabled')


        # Services
        self.get_firmware_srv = self.create_service(
            GetFirmware,
            'get_firmware',
            self.get_firmware_handler,
        )

        # Topics
        self.clients_pub = self.create_publisher(
            ClientList,
            'clients',
            qos_profile=qos_profile_sensor_data
        )
        self.client_status_thread = threading.Thread(
            target=self.client_status_thread_fn
        )
        self.client_status_thread.start()

        if self.enable_gps_param.value:
            self.navsat_fix_pub = self.create_publisher(
                NavSatFix,
                'gps_0/fix',
                qos_profile=qos_profile_sensor_data,
            )
            # start a background thread to publish the navsat fix data at 1Hz
            self.navsat_thread = threading.Thread(
                target=self.navsat_thread_fn,
            )
            self.navsat_thread.start()

    def wait_for_host(self):
        """
        Wait for host to come up.

        Wait until the host is actually online before we try to contact it.
        This reduces http related errors
        """
        # ping syntax is different on Windows than Linux, so set the command accordingly
        if os.name == 'nt':
            cmd = f'ping -W 5 -n 1 {self.ip_address}'.split()
        else:
            cmd = f'ping -W 5 -c 1 {self.ip_address}'.split()

        self.get_logger().info(f'Waiting until {self.ip_address} is online...')
        host_alive = subprocess.call(cmd) == 0  # noqa: S603
        rate = self.create_rate(1)
        while not host_alive:
            rate.sleep()
            host_alive = subprocess.call(cmd) == 0  # noqa: S603

        self.get_logger().info(f'{self.ip_address} is now online')

    def login(self):
        success = True
        url = f'https://{self.ip_address}/api/login'
        content = f"""{{
            "username": "{self.username}",
            "password": "{self.password}"
        }}""".encode()
        self.get_logger().info(f'Logging in as user "{self.username}"...')
        try:
            http_resp = self.session.post(
                url,
                data=content,
                headers=self.http_headers,
                verify=False,
            )
            data = json.loads(http_resp.content.decode())
            if data['stat'] != 'ok':
                success = False
                self.get_logger().warn('Login failed')
            else:
                self.get_logger().info('Login succeeded')

        except Exception as err:
            self.get_logger().error(f'Login failed with error: {err}')
            success = False

        return success

    def get_firmware_handler(self, request, result):
        get_url = f'https://{self.ip_address}/api/info.firmware'
        try:
            http_resp = self.session.get(
                get_url,
                headers=self.http_headers,
                verify=False,
            )

            data = json.loads(http_resp.content.decode())
            order = data['response']['order']

            for n in order:
                fw_json = data['response'][f'{n}']
                fw = Firmware()
                fw.version = fw_json['version']
                fw.bootable = fw_json['bootable']
                fw.in_use = fw_json['inUse']
                result.firmwares.append(fw)
        except Exception as err:
            self.get_logger().warning(f'Error querying firmware: {err}')
            result.firmwares = []

        return result

    def client_status_thread_fn(self):
        rate = self.create_rate(1)
        while rclpy.ok():
            clients = ClientList()
            get_url = f'https://{self.ip_address}/api/status.client?connectionType=ethernet wireless'
            try:
                http_resp = self.session.get(
                    get_url,
                    headers=self.http_headers,
                    verify=False,
                )
                data = json.loads(http_resp.content.decode())

                for client_json in data['response']['list']:
                    client = Client()
                    client.ip_address = client_json.get('ip', '')
                    client.connection_type = client_json.get('connectionType', 'other')
                    client.name = client_json.get('name', '')
                    client.mac = client_json.get('mac', '')
                    client.bssid = client_json.get('bssid', '')
                    client.essid = client_json.get('essid', '')
                    client.active = client_json.get('active', False)
                    client.vlan_id = client_json.get('vlanId', -1)

                    client.lease = Lease()
                    client.lease.expires_in = client_json.get('lease', {}).get('expiresIn', 0)
                    client.lease.type = client_json.get('lease', {}).get('type', '')

                    client.signal_strength = Signal()
                    client.signal_strength.value = client_json.get('signalStrength', {}).get('value', 0)
                    client.signal_strength.unit = client_json.get('signalStrength', {}).get('unit', '')

                    client.signal_details = SignalDetails()
                    client.signal_details.strength = client_json.get('signal', {}).get('strength', 0)
                    client.signal_details.level = client_json.get('signal', {}).get('level', 0)

                    client.bandwidth = Bandwidth()
                    client.bandwidth.download = client_json.get('speed', {}).get('download', 0)
                    client.bandwidth.upload = client_json.get('speed', {}).get('upload', 0)
                    client.bandwidth.unit = client_json.get('speed', {}).get('unit', '')

                    clients.clients.append(client)

                self.clients_pub.publish
            except Exception as err:
                self.get_logger().warning(f'Failed to query client status: {err}')

            self.clients_pub.publish(clients)
            rate.sleep()

    def navsat_thread_fn(self):
        rate = self.create_rate(1)
        fix = NavSatFix()
        while rclpy.ok():
            get_url = f'https://{self.ip_address}/api/info.location'
            try:
                http_resp = self.session.get(
                    get_url,
                    headers=self.http_headers,
                    verify=False,
                )
                data = json.loads(http_resp.content.decode())

                self.get_logger().info(str(data))

                if data['stat'] == 'fail':
                    self.get_logger().warn('Failed to read GPS data')
                    fix.latitude = math.nan
                    fix.longitude = math.nan
                    fix.altitude = math.nan
                elif not data['response']['gps']:
                    self.get_logger().warn('GPS data is invalid')
                    fix.latitude = math.nan
                    fix.longitude = math.nan
                    fix.altitude = math.nan
                else:
                    fix.latitude = data['response']['location']['latitude']
                    fix.longitude = data['response']['location']['longitude']
                    fix.altitude = data['response']['location']['altitude']
                fix.header.stamp = self.get_clock().now().to_msg()

                self.navsat_fix_pub.publish(fix)
            except Exception as err:
                self.get_logger().warning(f'Failed to query location: {err}')

            rate.sleep()


def main(args=None):
    rclpy.init()

    node_name = 'peplink_router_node'
    executor = MultiThreadedExecutor()
    node = PeplinkRouterNode(node_name)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
