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

from peplink_msgs.msg import Firmware
from peplink_msgs.srv import GetFirmware

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import requests
import requests.auth
import urllib3


class PeplinkRouterNode(Node):
    """
    The main interface class for the Peplink router.

    Provides basic access to the router state, allows users to change between wifi and cellular
    modes.
    """

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

        # Services
        self.get_firmware_srv = self.create_service(
            GetFirmware,
            'get_firmware',
            self.get_firmware_handler,
        )

        # Topics
        # TODO: any?

        # Actions
        # TODO: any?

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
    def http_auth(self):
        return requests.auth.HTTPBasicAuth(self.username, self.password)

    @property
    def http_timeout(self):
        return (3, 5)

    @property
    def http_headers(self):
        {
            'User-Agent': (
                'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko)'
                ' Chrome/111.0.0.0 Safari/537.36'
            ),
        }

    def get_firmware_handler(self, request, result):
        get_url = (
            f'https://{self.ip_address}/api/info.firmware'
        )
        try:
            http_resp = requests.get(
                get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout,
                verify=False,  # HTTPS certificate regularly fails, so just ignore it
            )

            # something of the form
            # {
            #   "stat": "ok",
            #   "response": {
            #     "1": {
            #       "version": "8.5.1 build 5714",
            #       "bootable": true,
            #       "inUse": true
            #     },
            #     "order": [
            #       1
            #     ]
            #   }
            # }
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


def main(args=None):
    rclpy.init()

    node_name = 'peplink_router_node'
    executor = MultiThreadedExecutor()
    node = PeplinkRouterNode(node_name)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()