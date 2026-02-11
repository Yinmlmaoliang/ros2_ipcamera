import os
import pathlib
import launch
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description with three camera components."""
    # Get the config directory
    config_dir = os.path.join(get_package_share_directory('ros2_ipcamera'), 'config')
    param_config = os.path.join(config_dir, "ipcamera.yaml")

    # Load parameters for all three cameras
    with open(param_config, 'r') as f:
        all_params = yaml.safe_load(f)

    camera1_params = all_params["camera1"]["ros__parameters"]
    camera2_params = all_params["camera2"]["ros__parameters"]
    camera3_params = all_params["camera3"]["ros__parameters"]

    # Create three camera nodes
    camera1_node = ComposableNode(
                    package='ros2_ipcamera',
                    plugin='ros2_ipcamera::IpCamera',
                    name='camera1',
                    parameters=[camera1_params])

    camera2_node = ComposableNode(
                    package='ros2_ipcamera',
                    plugin='ros2_ipcamera::IpCamera',
                    name='camera2',
                    parameters=[camera2_params])

    camera3_node = ComposableNode(
                    package='ros2_ipcamera',
                    plugin='ros2_ipcamera::IpCamera',
                    name='camera3',
                    parameters=[camera3_params])

    # Create a single container with all three cameras
    container = ComposableNodeContainer(
            name='multi_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera1_node,
                camera2_node,
                camera3_node
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
