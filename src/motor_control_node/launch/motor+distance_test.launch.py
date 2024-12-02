import os
from launch import LaunchDescription
from launch_ros.actions import Node

import time

def generate_launch_description():
    ld = LaunchDescription()

    motor_node = Node(
        package = 'motor_control_node',
        executable = 'motor_control',
        name = 'motor_control'
    )
    '''    aruco_node = Node(
        package = 'aruco_publisher',
        executable = 'aruco_detector',
        name = 'aruco_detector'
    )
    DCS_node = Node(
        package = 'distance_node',
        executable = 'distance_publisher',
        name = 'distance_publisher'
    )'''
    main_node = Node(
        package = 'main_package_robot',
        executable = 'main_node',
        name = 'main_node'
    )

    ld.add_action(motor_node)
    #ld.add_action(aruco_node)
    #ld.add_action(DCS_node)
    ld.add_action(main_node)

    return ld
