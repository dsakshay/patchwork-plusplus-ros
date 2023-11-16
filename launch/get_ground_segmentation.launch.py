#!/usr/bin/env python3

import os, yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("patchworkpp"), "config")
    
    params_file = os.path.join(config_path, "params_patchworkpp_node.yaml")
    default_rviz = os.path.join(get_package_share_directory("patchworkpp"), "rviz", "test_rviz.rviz")

    dict_params = {}

    with open(params_file, "r") as stream:
        try:
            dict_params = yaml.safe_load(stream)
        except yaml.YAMLError as err:
            print("Could not load YAML file. Error: ", err)

    patchwork_params = dict_params["patchworkpp_2"]["ros__parameters"]

    # ground segmentation node
    patchwork_node = Node(
        package="patchworkpp",
        executable="patchworkpp_2",
        name="patchworkpp_2",
        parameters=[patchwork_params],
        output="screen",
    )

    occupancy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("terra_launch_configs"), "launch", "modules", "timmy", "gen_occupancy_grid.launch.py"))
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["--display-config", default_rviz],
    )

    ld = LaunchDescription()

    ld.add_action(patchwork_node)
    ld.add_action(occupancy_launch)
    ld.add_action(rviz_node)

    return ld
