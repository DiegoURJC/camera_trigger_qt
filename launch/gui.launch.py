import os
import argparse
import sys
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gui_node = Node(
        package="camera_trigger_qt",
        executable="camera_trigger_qt",
        name="camera_trigger_qt",
        output="screen",
    )
    return LaunchDescription([gui_node])