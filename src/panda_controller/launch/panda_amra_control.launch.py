from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # get URDF file path
    urdf_file = PathJoinSubstitution(
        [FindPackageShare('panda_description'), 'urdf', 'panda.urdf']
    )

    # processing URDF 
    robot_description = {'robot_description': Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file
    ])}

    obstacle_data_file = PathJoinSubstitution(
        [FindPackageShare('amra_star'), 'maps', 'Testmap.map']
    )
    # beigin robot_state_publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Beigin pose_actuator_server
    action_server_node = Node(
        package='panda_controller',  
        executable='pose_actuator_server', 
        name='pose_actuator_server',
        output='screen'
    )

    # init and goal pose
    initial_position = [0.15, 0.1125, 0.0875]  # initial_position x, y, z
    initial_orientation = [0.707, 0.707, 0.0, 0.0]  # initial_orientation w, x, y, z
    goal_position = [0.6, 0.125, 0.1]  # goal position x, y, z
    goal_orientation = [0.707, 0.0, 0.707, 0.0]  # goal_orientation w, x, y, z

    # Begin client，add parameters
    action_client_node = Node(
        package='panda_controller',  
        executable='pose_actuator_client',  
        name='pose_actuator_client',
        output='screen',
        parameters=[
            {'scale_sensor_robot': 80.0},
            {'panda_description': urdf_file},
            {'map_file_path': obstacle_data_file},
            {'start_position': initial_position},
            {'start_orientation': initial_orientation},
            {'goal_position': goal_position},
            {'goal_orientation': goal_orientation}
        ]
    )

    # initialize RViz2 (default.rviz)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('panda_description'), 'rviz', 'with_obstacles.rviz']
    )



    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    obstacle_publisher_node = Node(
        package='panda_controller',
        executable='obstacle_publisher',
        name='obstacle_publisher',
        output='screen',
        parameters=[
            {'scale_sensor_robot': 80.0},
            {'start_position':initial_position},
            {'goal_position': goal_position},
            {'file_path': obstacle_data_file}
        ]
    )

    trajectory_publisher_node = Node(
        package='panda_controller',  
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        action_server_node,
        action_client_node,
        rviz_node,
        obstacle_publisher_node,
        trajectory_publisher_node
    ])
