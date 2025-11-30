from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def target_move_fn(context, *args, **kwargs):
    # start.launch.py와 동일하게 model / gripper 사용
    model_value = LaunchConfiguration('model').perform(context)
    gripper_value = LaunchConfiguration('gripper').perform(context)

    # dsr_moveit_config_<model> 패키지 이름 구성
    package_name = f"dsr_moveit_config_{model_value}"
    print("[target_move] MoveIt Config Package:", package_name)

    # MoveIt 설정 생성 (start.launch.py의 rviz_and_move_group_fn과 거의 동일)
    moveit_config = (
        MoveItConfigsBuilder(model_value, "robot_description", package_name)
        .robot_description(file_path=f"config/{model_value}.urdf.xacro")
        .robot_description_semantic(
            file_path="config/dsr.srdf.xacro",
            mappings={'gripper': gripper_value}
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp"],
            default_planning_pipeline="ompl",
            load_all=False
        )
        .to_moveit_configs()
    )

    common_params = [
        moveit_config.to_dict(),  # 여기 안에 robot_description / robot_description_semantic 다 들어감
    ]

    # move_group은 start.launch.py에서 실행되므로 여기서는 target_move만 실행
    target_move_node = Node(
        package="target_move",
        executable="target_move_node",
        output="screen",
        parameters=common_params,
    )

    return [target_move_node]


def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument(
            'model',
            default_value='e0509',   # e0509 모델 사용
            description='ROBOT_MODEL'
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='none',
            description='GRIPPER (none|robotiq_2f85)'
        ),
    ]

    target_move_action = OpaqueFunction(function=target_move_fn)

    return LaunchDescription(ARGUMENTS + [target_move_action])
