from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtém o diretório de compartilhamento do pacote MAVROS
    mavros_share_dir = get_package_share_directory('mavros')
    
    # Define os caminhos completos para os arquivos YAML
    config_yaml = os.path.join(mavros_share_dir, 'launch', 'px4_config.yaml')
    pluginlists_yaml = os.path.join(mavros_share_dir, 'launch', 'px4_pluginlists.yaml')

    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': 'serial:///dev/ttyACM0:57600', 
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
            }, 
            config_yaml, 
            pluginlists_yaml],
        )
    ])
