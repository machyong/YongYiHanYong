# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass

from isaaclab_tasks.manager_based.project import mdp
from isaaclab_tasks.manager_based.project.lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort: skip


# -----------------------------------------------------------------------------
# Franka joint-position lift env (tableware 버전)
# -----------------------------------------------------------------------------

@configclass
class FrankaCubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # parent 설정 먼저
        super().__post_init__()

        # env 개수 / 간격 (원하면 여기서 override)
        # self.scene.num_envs = 4
        # self.scene.env_spacing = 2.5

        # 1) 로봇 설정: Franka
        self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # 2) 액션 설정: Franka 관절 제어
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["panda_joint.*"],
            scale=0.5,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["panda_finger.*"],
            open_command_expr={"panda_finger_.*": 0.04},
            close_command_expr={"panda_finger_.*": 0.0},
        )

        # 3) EE 프레임 (object_pose 커맨드 기준 링크)
        self.commands.object_pose.body_name = "panda_hand"

        # 4) Dish (접시)
        self.scene.dish = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Dish",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.5, 0.0, 0.0],  # 테이블 위 살짝 띄워놓기 (필요시 조정)
                rot=[1.0, 0.0, 0.0, 0.0],
            ),
            spawn=UsdFileCfg(
                usd_path="/home/jw/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/project/assets/dish.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # 5) Spoon (숟가락)
        self.scene.spoon = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Spoon",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.45, 0.05, 0.06],   # 접시 옆
                rot=[1.0, 0.0, 0.0, 0.0],
            ),
            spawn=UsdFileCfg(
                usd_path="/home/jw/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/project/assets/spoon.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=8,
                    solver_velocity_iteration_count=1,
                    disable_gravity=False,
                ),
            ),
        )

        # 6) Fork (포크)
        self.scene.fork = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Fork",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.55, -0.05, 0.06],
                rot=[1.0, 0.0, 0.0, 0.0],
            ),
            spawn=UsdFileCfg(
                usd_path="/home/jw/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/project/assets/fork.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=8,
                    solver_velocity_iteration_count=1,
                    disable_gravity=False,
                ),
            ),
        )

        # 7) Frame Transformer (EE frame)
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"

        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.1034],  # 기존 Franka lift 예제와 동일
                    ),
                ),
            ],
        )


@configclass
class FrankaCubeLiftEnvCfg_PLAY(FrankaCubeLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # play용: env 수 늘리고, obs corruption 끔
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False



# -----------------------------------------------------------------------------
# e0509 joint-position lift env (커스텀)
# -----------------------------------------------------------------------------

# @configclass
# class E0509CubeLiftEnvCfg(LiftEnvCfg):
#     def __post_init__(self):
#         # post init of parent
#         super().__post_init__()

#         # 1) Set e0509 as robot (local USD)
#         self.scene.robot = E0509_CFG
#         self.scene.num_envs = 4
#         self.scene.env_spacing = 2.5
        

#         # 2) Arm joint position action (joint_1 ~ joint_6)
#         self.actions.arm_action = mdp.JointPositionActionCfg(
#             asset_name="robot",
#             joint_names=["joint_.*"],  # joint_1 ~ joint_6 모두 매칭
#             scale=0.5,
#             use_default_offset=True,
#         )

#         # 3) Gripper action (4 DOF: l1, l2, r1, r2)
#         self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
#             asset_name="robot",
#             joint_names=[
#                 "rh_l1",
#                 "rh_l2",
#                 "rh_p12_rn",
#                 "rh_r2",
#             ],
#             open_command_expr={
#                 "rh_l1": 0.04,
#                 "rh_l2": 0.04,
#                 "rh_p12_rn": 0.04,
#                 "rh_r2": 0.04,
#             },
#             close_command_expr={
#                 "rh_l1": 0.0,
#                 "rh_l2": 0.0,
#                 "rh_p12_rn": 0.0,
#                 "rh_r2": 0.0,
#             },
#         )

#         # 4) EE 링크 이름 설정
#         self.commands.object_pose.body_name = "rh_p12_rn_base"

#         # 5) Cube object 설정 (Franka 버전과 동일)
#         self.scene.object = RigidObjectCfg(
#             prim_path="{ENV_REGEX_NS}/Object",
#             init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[1, 0, 0, 0]),
#             spawn=UsdFileCfg(
#                 usd_path="/home/jw/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/project/assets/dish.usd",
#                 scale=(1.0, 1.0, 1.0),
#                 rigid_props=RigidBodyPropertiesCfg(
#                     solver_position_iteration_count=16,
#                     solver_velocity_iteration_count=1,
#                     max_angular_velocity=1000.0,
#                     max_linear_velocity=1000.0,
#                     max_depenetration_velocity=5.0,
#                     disable_gravity=False,
#                 ),
#             ),
#         )

#         # 6) FrameTransformer: EE 프레임을 rh_p12_rn_base로 사용
#         marker_cfg = FRAME_MARKER_CFG.copy()
#         marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
#         marker_cfg.prim_path = "/Visuals/FrameTransformer"
#         self.scene.ee_frame = FrameTransformerCfg(
#             prim_path="{ENV_REGEX_NS}/Robot/e0509/base_link",
#             debug_vis=False,
#             visualizer_cfg=marker_cfg,
#             target_frames=[
#                 FrameTransformerCfg.FrameCfg(
#                     prim_path="{ENV_REGEX_NS}/Robot/e0509/rh_p12_rn_base",
#                     name="end_effector",
#                     offset=OffsetCfg(
#                         pos=[0.0, 0.0, 0.0],  # 필요하면 EE 기준 offset 나중에 조정
#                     ),
#                 ),
#             ],
#         )


# @configclass
# class E0509CubeLiftEnvCfg_PLAY(E0509CubeLiftEnvCfg):
#     def __post_init__(self):
#         # post init of parent
#         super().__post_init__()
#         # make a smaller scene for play
#         self.scene.num_envs = 4
#         self.scene.env_spacing = 2.5
#         # disable randomization for play
#         self.observations.policy.enable_corruption = False
