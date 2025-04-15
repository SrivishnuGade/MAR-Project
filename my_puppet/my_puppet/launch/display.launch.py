from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('my_puppet')

    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'puppet.urdf')

    # Check if the URDF file exists, else try alternative filenames
    if not os.path.exists(urdf_file):
        potential_files = [
            os.path.join(pkg_dir, 'urdf', 'puppet.urdf.xacro'),
            os.path.join(pkg_dir, 'urdf', 'model.urdf'),
            os.path.join(pkg_dir, 'urdf', 'model.urdf.xacro')
        ]
        for file in potential_files:
            if os.path.exists(file):
                urdf_file = file
                break

    # Process URDF or XACRO
    if urdf_file.endswith('.xacro'):
        robot_description = Command(['xacro', urdf_file])  # ✅ Fixed spacing
    else:
        with open(urdf_file, 'r') as file:
            robot_description = file.read()

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # RViz Configuration
    rviz_config_file = os.path.join(pkg_dir, 'config', 'puppet.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = None  # ✅ Fixed default case

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if rviz_config_file else [],
        output='screen'
    )

    # Puppet Controller
    puppet_controller_node = Node(
        package='my_puppet',
        executable='puppet_controller',
        name='puppet_controller',
        output='screen'
    )

    # Print URDF joints command
    print_joints_cmd = ExecuteProcess(
        cmd=['echo', 'URDF joints in the model:'],
        output='screen'
    )

    # Launch all nodes
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time if true'),
        print_joints_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        puppet_controller_node
    ])

