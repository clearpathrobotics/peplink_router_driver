# Peplink Router Driver

This repository contains a ROS 2 driver for [Peplink mobile routers](https://www.peplink.com/products/mobile-routers/)

## Usage

Start the driver with
```bash
ros2 launch peplink_router_driver peplink_router.launch.py ip_address:=192.168.50.1 username:=admin password:=password
```
substituting the `ip_address` and `password` fields as needed.

The `namespace` launch argument can be used to set the namespace for the node's topics and services, e.g.
```bash
ros2 launch peplink_router_driver peplink_router.launch.py ip_address:=192.168.50.1 username:=admin password:=password namespace:=/robot_ns/wifi/router_0
```
