cmake_minimum_required(VERSION 3.5)
project(peplink_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(wireless_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    # action/

    msg/Bandwidth.msg
    msg/Client.msg
    msg/ClientList.msg
    msg/Firmware.msg
    msg/Lan.msg
    msg/LanList.msg
    msg/Lease.msg

    srv/GetFirmware.srv

    DEPENDENCIES wireless_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
