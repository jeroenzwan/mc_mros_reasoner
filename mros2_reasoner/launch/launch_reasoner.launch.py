# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    mros2_reasoner_bringup_dir = get_package_share_directory('mros2_reasoner')
    modes_observer_dir = get_package_share_directory('mros_modes_observer')
     
    
    # Create the launch configuration variables
    model_file = LaunchConfiguration('model_file')
    tomasys_file = LaunchConfiguration('tomasys_file')
    desired_configuration = LaunchConfiguration('desired_configuration')
    nfr_energy = LaunchConfiguration('nfr_energy')
    nfr_safety = LaunchConfiguration('nfr_safety')
    components_file = LaunchConfiguration('components_file')

    # Declare the launch arguments
    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(mros2_reasoner_bringup_dir, 'MROS_ontology.owl'),
        description='File name for the ontology model')

    declare_tomasys_file_cmd = DeclareLaunchArgument(
        'tomasys_file',
        default_value=os.path.join(mros2_reasoner_bringup_dir, 'tomasys.owl'),
        description='File name for the Tomasys ontology')

    declare_desired_configuration_cmd = DeclareLaunchArgument(
        'desired_configuration',
        default_value='f_normal_mode',
        description='Desired inital configuration (system mode)')

    declare_nfr_energy_cmd = DeclareLaunchArgument(
        'nfr_energy',
        default_value='0.5',
        description='Required value for Energy NFR')

    declare_nfr_safety_cmd = DeclareLaunchArgument(
        'nfr_safety',
        default_value='0.5',
        description='Required value for Safety NFR')
    
    declare_components_file = DeclareLaunchArgument(
        'components_file',
        default_value=os.path.join(modes_observer_dir, 'params', 'components.yaml'),
        description='File name for the obsreved components yaml')


    bringup_reasoner_cmd = Node(
        package='mros2_reasoner',
        executable='mros2_reasoner_node',
        name='mros2_reasoner_node',
        output='screen',
        parameters=[{
            'model_file': model_file,
            'tomasys_file': tomasys_file,
            'desired_configuration': desired_configuration,
            'nfr_energy': nfr_energy,
            'nfr_safety': nfr_safety,
            }],
    )

    #
    modes_observer_node = Node(
    package='mros_modes_observer',
    executable='modes_observer_node',
    parameters=[{'componentsfile': components_file}],
    output='screen')


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_model_file_cmd)
    ld.add_action(declare_tomasys_file_cmd)
    ld.add_action(declare_desired_configuration_cmd)
    ld.add_action(declare_nfr_energy_cmd)
    ld.add_action(declare_nfr_safety_cmd)
    ld.add_action(declare_components_file)

    # Add the actions to launch the reasoner node
    ld.add_action(bringup_reasoner_cmd)

    # Add system modes manager
    ld.add_action(modes_observer_node)


    return ld
