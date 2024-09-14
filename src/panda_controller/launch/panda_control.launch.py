from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 获取URDF和RViz配置文件路径
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('panda_description'),
        'urdf',
        'panda.urdf'
    ])
    
    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory('panda_description'),
        'rviz',
        'default.rviz'
    ])
    
    # 声明URDF模型路径参数
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=urdf_file,
        description='Absolute path to robot urdf file'
    )
    
    # 声明RViz配置文件路径参数
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=rviz_config_file,
        description='Absolute path to rviz config file'
    )
    
    # 读取URDF文件内容
    robot_description_content = ParameterValue(
        Command(['cat ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 配置robot_state_publisher节点，仅发布TF，不发布JointState
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )
    
    # 配置pose_subscriber节点作为唯一的JointState发布者
    pose_subscriber_node = Node(
        package='panda_controller',
        executable='pose_subscriber',
        name='pose_subscriber',
        output='screen'
    )
    
    # 配置RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )
    
    # 配置target_pose_publisher节点
    target_pose_publisher_node = Node(
        package='panda_controller',
        executable='target_pose_publisher',
        name='target_pose_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        model_arg,                     # 声明URDF文件路径参数
        rviz_arg,                      # 声明RViz配置文件路径参数
        robot_state_publisher_node,    # 启动robot_state_publisher节点
        pose_subscriber_node,          # 启动pose_subscriber节点，作为唯一的JointState发布者
        rviz_node,                     # 启动RViz节点
        target_pose_publisher_node     # 启动target_pose_publisher节点
    ])
