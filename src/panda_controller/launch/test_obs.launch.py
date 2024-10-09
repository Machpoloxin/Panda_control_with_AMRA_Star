from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 启动 RViz2 节点，使用默认配置

    # initialize RViz2 (default.rviz)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('panda_description'), 'rviz', 'tes_obs.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[]
    )

    # 启动 obstacle_publisher 节点，用于发布障碍物
    obstacle_publisher_node = Node(
        package='panda_controller',
        executable='obstacle_publisher',
        name='obstacle_publisher',
        output='screen'
    )

    # 返回包含 RViz2 和障碍物发布器的节点
    return LaunchDescription([
        rviz_node,                   # 启动 RViz2
        obstacle_publisher_node      # 启动障碍物发布器
    ])
