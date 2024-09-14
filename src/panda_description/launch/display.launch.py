from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_tutorial_path = get_package_share_directory('panda_description')
    default_model_path = urdf_tutorial_path + '/urdf/panda.urdf'
    default_rviz_config_path = urdf_tutorial_path + '/rviz/default.rviz'

    # 声明URDF模型路径参数
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
                                      
    # 声明RViz配置文件路径参数
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # 使用ParameterValue和Command读取URDF文件内容
    robot_description_config = ParameterValue(
        Command(['cat ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 配置robot_state_publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}]
    )

    # # 配置joint_state_publisher节点
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen'
    # )

    # 配置RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        #output='screen',
        #arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        model_arg,                     # 确保参数声明在使用之前
        rviz_arg,                      # 确保参数声明在使用之前
        robot_state_publisher_node,    # 使用已声明的参数
        #joint_state_publisher_node,
        rviz_node
    ])
    
    
#if __name__ == '__main__':
#    generate_launch_description()
