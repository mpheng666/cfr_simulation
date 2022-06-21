from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                        description='The world path, by default is empty.world'),
    DeclareLaunchArgument('cfr_world_path', default_value='',
                          description='The world path, by default is empty.world'),
]

def generate_launch_description():

    cfr_world_path = PathJoinSubstitution(
                    [FindPackageShare("cfr_gazebo"), "worlds", "cfr_boundary.world"]
    )
    # Launch args
    world_path = LaunchConfiguration('world_path')
    prefix = LaunchConfiguration('prefix')

    config_cfr_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("cfr_control"), "config", "control.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("cfr_description"), "urdf", "cfr.urdf.xacro"]
            ),
            " ",
            "name:=cfr",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_cfr_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    spawn_cfr_velocity_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['forward_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_cfr_velocity_controller starts after spawn_joint_state_broadcaster
    controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_cfr_velocity_controller],
        )
    )
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             cfr_world_path],
            #  prefix=['xterm -e gdb -ex run --args'],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_cfr',
        arguments=['-entity',
                   'cfr',
                   '-topic',
                   'robot_description'],
        output='screen',
    )

    # Launch cfr_control/control.launch.py which is just robot_localization.
    launch_cfr_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("cfr_control"), 'launch', 'control.launch.py'])))

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(controller_spawn_callback)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    # ld.add_action(launch_cfr_control)

    return ld
