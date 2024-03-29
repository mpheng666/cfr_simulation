import os
import launch
import launch_ros
import yaml

from ament_index_python import get_package_share_directory

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    network = os.path.join(
    get_package_share_directory('cfr_socket_comm'),
    'config',
    'network.yaml'
    )
       
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            name='cfr_feedback_client',
            package='cfr_socket_comm',
            executable='cfr_feedback_client_node',
            output='screen',
            namespace='cfr',
            parameters = [network],
        )
    ])