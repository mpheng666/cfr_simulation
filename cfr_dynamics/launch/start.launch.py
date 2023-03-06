import os
import launch
import launch_ros
import yaml

from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    config = os.path.join(
    get_package_share_directory('cfr_dynamics'),
    'config',
    'environment.yaml'
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            name='cfr_dynamics',
            package='cfr_dynamics',
            executable='cfr_dynamics_node',
            output='screen',
            namespace='cfr',
            parameters = [config],
            remappings=[
            ('/cfr/odom', '/odom'),
            ('/cfr/motor_actuation', 'cfr_mpc/motor_actuation')]
        )
    ])