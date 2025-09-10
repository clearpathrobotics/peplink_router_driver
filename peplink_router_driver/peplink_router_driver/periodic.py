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
import threading

import rclpy


class PeriodicCheck:
    """
    Calls an endpoint on the Peplink router at regular intervals.

    Only supports HTTP(S) GET endpoints.

    :param nh: The PeplinkRouterNode that owns this checker
    :param url: The complete URL to GET
    :param rate: The update rate in Hz
    :param callback: A function to send the HTTP response body to. Must accept a dict object as the argument
    """

    def __init__(
        self,
        nh,
        url: str,
        rate: float,
        callback: callable,
    ):
        self.thread = threading.Thread(target=self.run_in_background)
        self.callback = callback
        self.url = url
        self.nh = nh
        self.rate = rate

        self.thread.start()

    def get_json(self):
        http_resp = self.nh.session.get(
                    self.url,
            headers=self.nh.http_headers,
            verify=False,
        )
        json_data = json.loads(http_resp.content.decode())
        # self.nh.get_logger().debug(f'{json_data}')
        return json_data


    def run_in_background(self):
        rate = self.nh.create_rate(self.rate)

        while rclpy.ok():
            try:
                json_data = self.get_json()
                if json_data.get('code', 200) == 401:
                    self.nh.get_logger().warn(f'Authentication failed for {self.url}. Reauthenticating and trying again')
                    self.nh.login()
                    json_data = self.get_json()
                self.callback(json_data)

            except Exception as err:
                self.nh.get_logger().warning(f'Failed to query URL {self.url}: {err}')

            rate.sleep()
