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

from peplink_router_driver.checker import(
    Authentication,
    PeplinkCheck,
)

import threading

import rclpy


class PeriodicCheck(PeplinkCheck):
    """
    Calls an endpoint on the Peplink router at regular intervals.

    Only supports HTTP(S) GET endpoints.

    :param nh: The PeplinkRouterNode that owns this checker
    :param url: The complete URL to GET
    :param rate: The update rate in Hz
    :param callback: A function to send the HTTP response body to. Must accept a dict object as the argument
    :param authentication:  Login credentials if this enpoint requires them
    """

    def __init__(
        self,
        nh,
        url: str,
        rate: float,
        callback: callable,
        authentication: Authentication=None,
    ):
        super().__init__(
            nh,
            url,
            authentication,
        )
        self.callback = callback
        self.rate = rate

        self.thread = threading.Thread(target=self.run_in_background)
        self.thread.start()

    def run_in_background(self):
        rate = self.nh.create_rate(self.rate)

        while rclpy.ok():
            try:
                json_data = self.get_json()
                self.callback(json_data)

            except Exception as err:
                self.nh.get_logger().warning(f'Failed to query URL {self.url}: {err}')

            rate.sleep()
