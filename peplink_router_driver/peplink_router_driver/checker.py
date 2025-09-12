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
import requests
import urllib3


class Authentication:
    """
    Container for HTTP(S) login credentials and a persistent session.

    :param nh:  The node that owns this object
    :param ip_address:  The remote IP address we're logging into
    :param username:  The login username
    :param password:  The login password
    """

    def __init__(self,
        nh,
        ip_address: str,
        username: str,
        password: str,
    ):
        self.nh = nh
        self.ip_address = ip_address
        self.username = username
        self.password = password
        self._session = requests.Session()

    def login(self):
        """
        Attempt to log into the server.

        :return: True if the attempt was successful, otherwise False
        """
        success = True
        url = f'https://{self.ip_address}/api/login'
        content = {
            'username': self.username,
            'password': self.password,
        }
        self.nh.get_logger().info(f'Logging in as user "{self.username}"...')
        try:
            http_resp = self.session.post(
                url,
                data=json.dumps(content).encode(),
                headers=self.nh.http_headers,
                verify=False,
            )
            data = json.loads(http_resp.content.decode())
            if data['stat'] != 'ok':
                success = False
                self.nh.get_logger().warn('Login failed')
            else:
                self.nh.get_logger().info('Login succeeded')

        except Exception as err:
            self.nh.get_logger().error(f'Login failed with error: {err}')
            success = False

        return success

    @property
    def session(self) -> requests.Session:
        return self._session


class PeplinkCheck:
    """
    Calls an endpoint on the Peplink router.

    :param nh: The PeplinkRouterNode that owns this checker
    :param url: The complete URL to GET
    :param authentication:  Login credentials if this enpoint requires them
    """

    def __init__(
        self,
        nh,
        url: str,
        authentication: Authentication=None,
    ):
        self.url = url
        self.nh = nh
        self.authentication = authentication

    def get_json(self, retry_auth: bool=True) -> dict:
        """
        Call the enpoint and return a dict representing the JSON response.

        :param retry_auth: If we get a 401 error, authenticate again and retry
        :return: A dict representing the JSON response from the server
        """
        if self.authentication is not None:
            http_resp = self.authentication.session.get(
                self.url,
                headers=self.nh.http_headers,
                verify=False,
            )
            json_data = json.loads(http_resp.content.decode())
            if retry_auth and json_data.get('code', 200) == 401:
                self.nh.get_logger().warning(f'Failed to fetch data from {self.url}. Reauthenticating.')
                self.authentication.login()
                json_data = self.get_json(retry_auth=False)
        else:
            http_resp = requests.get(
                self.url,
                headers=self.nh.http_headers,
                verify=False,
            )
            json_data = json.loads(http_resp.content.decode())

        return json_data

    def post_json(self, payload: dict, retry_auth: bool=True) -> dict:
        """
        Call the enpoint and return a dict representing the JSON response.

        :param payload: The JSON payloat to POST
        :param retry_auth: If we get a 401 error, authenticate again and retry
        :return: A dict representing the JSON response from the server
        """
        if self.authentication is not None:
            http_resp = self.authentication.session.post(
                self.url,
                data=json.dumps(payload).encode(),
                headers=self.nh.http_headers,
                verify=False,
            )
            json_data = json.loads(http_resp.content.decode())
            if retry_auth and json_data.get('code', 200) == 401:
                self.nh.get_logger().warning(f'Failed to fetch data from {self.url}. Reauthenticating.')
                self.authentication.login()
                json_data = self.post_json(payload, retry_auth=False)
        else:
            http_resp = requests.post(
                self.url,
                data=json.dumps(payload).encode(),
                headers=self.nh.http_headers,
                verify=False,
            )
            json_data = json.loads(http_resp.content.decode())

        return json_data
