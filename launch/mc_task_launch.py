from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node



def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='mixied_critic_config',
            default_value=[],
            description="Description for mixed_critic_node"
        ),
        Node(
            package='mixed_critic',
            executable='lo_node',
            name='lo_node_executor',
            parameters=[]
        ),
        Node(
            package='mixed_critic',
            executable='hi_node',
            name='hi_node_executor',
            parameters=[]
        )
    ])