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

    map_publisher_node = Node(
        package="map_publisher",
        executable="map_publisher_node"
    )

    path_planner_node = Node(
        package="PathPlanners",
        executable="planner_test",
    )

    # agent_node = Node(
    #     package="agent",
    #     executable="service_test",
    # )
    
    return LaunchDescription([
        marker_node,
        rviz2_node,
        map_publisher_node,
        path_planner_node
    ])