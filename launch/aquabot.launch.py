from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Argument for the world to load
    default_world_name = 'aquabot_regatta'
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_name,
        description='World name'
    )
    ld.add_action(world_arg)

    # Include the launch file for simulation
    aquabot_competition_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('aquabot_gz'),
                         'launch/competition.launch.py')
        ),
        launch_arguments={'world': 'aquabot_windturbines_easy', 'competition_mode': 'true'}.items()
    )

    # Add example node (aquabot_node)
    aquabot_example_node = Node(
        package='aquabot_example',
        executable='aquabot_node',
        output='screen'
    )

    # Add QR code-related nodes
    qr_code_detector_node = Node(
        package='qr_code_V2_pkg',       
        executable='qr_code_detector',  
        output='screen'                 
    )

    qr_code_reader_node = Node(
        package='qr_code_V2_pkg',       
        executable='qr_code_reader',  
        output='screen'                 
    )

    wind_turbine_perimeter_node = Node(
        package='qr_code_V2_pkg',       
        executable='wind_turbine_perimeter',  
        output='screen'                 
    )

    # Include the C++ node from boat_mover
    boat_mover_node = Node(
        package='boat_mover',
        executable='mover_node',
        name='mover_node',
        output='screen'
    )

    # Add all actions to LaunchDescription
    ld.add_action(aquabot_competition_launch_file)
    ld.add_action(aquabot_example_node)
    ld.add_action(wind_turbine_perimeter_node)
    ld.add_action(boat_mover_node)

    return ld
