cmake_minimum_required(VERSION 3.5)
project(peplink_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        # action/

        msg/Bandwidth.msg
        msg/Client.msg
        msg/ClientList.msg
        msg/Firmware.msg
        msg/Lease.msg
        msg/Signal.msg
        msg/SignalDetails.msg

        srv/GetFirmware.srv
)

ament_package()
