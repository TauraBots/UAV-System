#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    # Obtenha o diretório compartilhado do pacote 'base_controller'
    base_controller_share_dir = FindPackageShare('base_controller').find('base_controller')
    base_controller_params = os.path.join(base_controller_share_dir, 'config', 'mavros_config.yaml')

    # Configurações de portas e taxas de transmissão
    port = LaunchConfiguration('port', default='/dev/ttyTHS1')
    baudrate = LaunchConfiguration('baudrate', default='921600')

    # Obtenha o diretório compartilhado do pacote 'mavros' e localize o arquivo de lançamento 'apm.launch'
    mavros_launch_file = os.path.join(FindPackageShare('mavros').find('mavros'), 'launch', 'apm.launch')

    return LaunchDescription([
        SetEnvironmentVariable('name', 'value'),
        
        Node(
            package='base_controller',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[base_controller_params]
        ),
        
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch_file),
            launch_arguments={'fcu_url': [port, ':', baudrate]}.items()
        )
    ])
