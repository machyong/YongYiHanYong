import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',       default_value='dsr01'),
        DeclareLaunchArgument('host',       default_value='127.0.0.1'),
        DeclareLaunchArgument('port',       default_value='12345'),
        DeclareLaunchArgument('mode',       default_value='virtual'),
        DeclareLaunchArgument('model',      default_value='m1013'),
        DeclareLaunchArgument('color',      default_value='white'),
        DeclareLaunchArgument('gui',        default_value='false'),
        DeclareLaunchArgument('gz',         default_value='false'),
        DeclareLaunchArgument('rt_host',    default_value='192.168.137.50'),
        DeclareLaunchArgument('remap_tf',   default_value='false'),
    ]
    mode = LaunchConfiguration('mode')
    name = LaunchConfiguration('name')
    xacro_path = os.path.join(
        get_package_share_directory('dsr_description2'),
        'xacro'
    )
    # URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('dsr_description2'),
                    'xacro',
                    LaunchConfiguration('model'),
                ]
            ),
            '.urdf.xacro',
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    # controller yaml
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('dsr_controller2'),
            'config',
            'dsr_controller2.yaml',
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('dsr_description2'), 'rviz', 'default.rviz']
    )
    # ------------------ DSR 설정/에뮬레이터 ------------------
    set_config_node = Node(
        package='dsr_bringup2',
        executable='set_config',
        namespace=name,
        parameters=[
            {'name':    name},
            {'rate':    100},
            {'standby': 5000},
            {'command': True},
            {'host':    LaunchConfiguration('host')},
            {'port':    LaunchConfiguration('port')},
            {'mode':    mode},
            {'model':   LaunchConfiguration('model')},
            {'gripper': 'none'},
            {'mobile':  'none'},
            {'rt_host': LaunchConfiguration('rt_host')},
        ],
        output='screen',
    )
    run_emulator_node = Node(
        package='dsr_bringup2',
        executable='run_emulator',
        namespace=name,
        parameters=[
            {'name':    name},
            {'rate':    100},
            {'standby': 5000},
            {'command': True},
            {'host':    LaunchConfiguration('host')},
            {'port':    LaunchConfiguration('port')},
            {'mode':    mode},
            {'model':   LaunchConfiguration('model')},
            {'gripper': 'none'},
            {'mobile':  'none'},
            {'rt_host': LaunchConfiguration('rt_host')},
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output='screen',
    )
    # ------------------ ros2_control_node ------------------
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=name,
        parameters=[robot_description, robot_controllers],
        output='both',
    )
    # ------------------ Robot State Publisher / RViz ------------------
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name,
        output='both',
        parameters=[{
            'robot_description': Command([
                'xacro ', xacro_path, '/',
                LaunchConfiguration('model'),
                '.urdf.xacro color:=', LaunchConfiguration('color')
            ])
        }],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=name,
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )
    original_tf_nodes = GroupAction(
        actions=[robot_state_pub_node, rviz_node],
        condition=UnlessCondition(LaunchConfiguration('remap_tf')),
    )
    remapped_tf_nodes = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            robot_state_pub_node,
            rviz_node,
        ],
        condition=IfCondition(LaunchConfiguration('remap_tf')),
    )
    # ------------------ Controller Spawners ------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        namespace=name,
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        output='screen',
    )
    dsr_hw_spawner = Node(
        package='controller_manager',
        namespace=name,
        executable='spawner',
        arguments=['dsr_controller2', '-c', 'controller_manager'],
        output='screen',
    )
    # ★ 이게 우리가 RL/MoveIt에서 사용할 joint_trajectory_controller
    moveit_traj_spawner = Node(
        package='controller_manager',
        namespace=name,
        executable='spawner',
        arguments=['dsr_moveit_controller', '-c', 'controller_manager'],
        output='screen',
    )
    # 순서: joint_state_broadcaster → dsr_controller2 → dsr_moveit_controller
    delay_dsr_hw_after_js = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dsr_hw_spawner],
        )
    )
    delay_moveit_after_dsr_hw = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=dsr_hw_spawner,
            on_exit=[moveit_traj_spawner],
        )
    )
    # set_config가 끝난 후 ros2_control_node 실행 (virtual 연결 준비되고 나서)
    delay_control_after_config = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=set_config_node,
            on_exit=[control_node, joint_state_broadcaster_spawner],
        )
    )
    nodes = [
        set_config_node,
        run_emulator_node,
        original_tf_nodes,
        remapped_tf_nodes,
        delay_control_after_config,
        delay_dsr_hw_after_js,
        delay_moveit_after_dsr_hw,
    ]
    return LaunchDescription(ARGUMENTS + nodes)