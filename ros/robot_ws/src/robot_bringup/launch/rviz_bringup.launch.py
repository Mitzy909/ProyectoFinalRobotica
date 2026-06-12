#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
  description_path = get_package_share_directory("robot_description")

  urdf_path = os.path.join(description_path, "urdf", "robot_rrr.urdf")
  rviz_path = os.path.join(description_path, "rviz", "rviz.conf.rviz")

  with open(urdf_path, "r") as infp:
    robot_desc = infp.read()

  urdf_param = {"robot_description": robot_desc}

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", rviz_path]
  )

  robot_description_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[urdf_param]
  )

  joint_publisher_node = Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui"
  )

  return LaunchDescription([
    rviz_node,
    robot_description_node,
    joint_publisher_node
  ])