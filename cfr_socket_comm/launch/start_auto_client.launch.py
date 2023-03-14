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

    commands = os.path.join(
    get_package_share_directory('cfr_socket_comm'),
    'config',
    'auto_client_command.yaml'
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            name='cfr_auto_command_client',
            package='cfr_auto_command_client',
            executable='cfr_auto_command_client_node',
            output='screen',
            namespace='cfr',
            parameters = [commands],
            remappings=[('cfr_auto_command_client/cmd_vel', '/cfr/cfr_mpc/cmd_vel')]
        )
    ])