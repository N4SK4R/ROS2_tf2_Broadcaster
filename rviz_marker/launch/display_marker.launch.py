from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():

    rviz_config_path = os.path.join(get_package_share_path('rviz_marker'),
                                    'rviz', 'rviz_config.rviz')
    
    marker_node = Node(
        package="rviz_marker",
        executable="display_marker"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    path_planner_node = Node(
        package="agent",
        executable="planner_test"
    )
    
    agent_node_1= Node(
        package="agent",
        executable="service_test",
        arguments=['agent_1','0','0','0']
    )
    
    agent_node_2= Node(
        package="agent",
        executable="service_test",
        arguments=['agent_2','10','5','0']
    )
    
    agent_node_3= Node(
        package="agent",
        executable="service_test",
        arguments=['agent_3','1','0','0']
    )
    
    agent_node_4= Node(
        package="agent",
        executable="service_test",
        arguments=['agent_4','4','0','0']
    )
    
    return LaunchDescription([
        marker_node,
        rviz2_node,
        path_planner_node,
        agent_node_1,
        agent_node_2,
        agent_node_3,
        agent_node_4

    ])