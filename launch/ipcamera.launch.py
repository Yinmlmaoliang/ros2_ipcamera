import os
import pathlib
import launch
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description with multiple components."""
    # Get the config directory
    config_dir = os.path.join(get_package_share_directory('ros2_ipcamera'), 'config')
    param_config = os.path.join(config_dir, "ipcamera.yaml")
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["ipcamera"]["ros__parameters"]

    ipcamera_node = ComposableNode(
                    package='ros2_ipcamera',
                    plugin='ros2_ipcamera::IpCamera',
                    name='ipcamera',
                    parameters=[params])

    container = ComposableNodeContainer(
            name='container',
            namespace='ipcamera_container',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[ipcamera_node],
            output='screen',
    )

    return launch.LaunchDescription([container])
