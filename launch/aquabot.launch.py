from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Argument pour le monde à charger
    default_world_name = 'aquabot_regatta'
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_name,
        description='World name'
    )
    ld.add_action(world_arg)

    # Inclure le fichier de lancement pour la simulation
    aquabot_competition_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('aquabot_gz'),
                         'launch/competition.launch.py')
        ),
        launch_arguments={'world': 'aquabot_windturbines_easy', 'competition_mode': 'true'}.items()
    )

    # Ajouter le nœud d'exemple (aquabot_node)
    aquabot_example_node = Node(
        package='aquabot_example',
        executable='aquabot_node',
        output='screen'
    )

    # Ajouter le nœud de détection de QR code
    qr_code_detector_node = Node(
        package='qr_code_V2_pkg',       # Ton package
        executable='qr_code_detector',  # L'exécutable configuré dans setup.py
        output='screen'                 # Affiche les logs dans le terminal
    )

    qr_code_reader_node = Node(
        package='qr_code_V2_pkg',       # Ton package
        executable='qr_code_reader',  # L'exécutable configuré dans setup.py
        output='screen'                 # Affiche les logs dans le terminal
    )

    wind_turbine_perimeter_node = Node(
        package='qr_code_V2_pkg',       # Ton package
        executable='wind_turbine_perimeter',  # L'exécutable configuré dans setup.py
        output='screen'                 # Affiche les logs dans le terminal
    )


    # Ajouter toutes les actions à LaunchDescription
    ld.add_action(aquabot_competition_launch_file)
    ld.add_action(aquabot_example_node)
    ld.add_action(qr_code_detector_node)
    ld.add_action(qr_code_reader_node)
    ld.add_action(wind_turbine_perimeter_node)


    return ld
