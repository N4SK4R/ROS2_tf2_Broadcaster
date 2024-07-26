import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('elevator_sim')
    world_file = os.path.join(pkg_dir, 'worlds', 'elevator_world.sdf')
    
    agent_pose_node_1= Node(
        package="local_planner",
        executable="pose_listener",
        arguments=['agent_1']
    )
    
    agent_pose_node_2= Node(
        package="local_planner",
        executable="pose_listener",
        arguments=['agent_2']
    )

    agent_pose_node_3= Node(
        package="local_planner",
        executable="pose_listener",
        arguments=['agent_3']
    )
    
    agent_pose_node_4= Node(
        package="local_planner",
        executable="pose_listener",
        arguments=['agent_4']
    )
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so'],
            output='screen'),
        agent_pose_node_1,
        agent_pose_node_2,
        agent_pose_node_3,
        agent_pose_node_4
    ])