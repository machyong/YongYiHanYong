# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import torch

from isaaclab.assets import RigidObjectCfg, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.project_lift import mdp
from isaaclab_tasks.manager_based.project_lift.lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
# from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort: skip


@configclass
class FrankaCubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.robot= ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=ArticulationCfg.InitialStateCfg(
                pos=[0, 0, -0.3135], 
                rot=[1, 0, 0, 0],
                joint_pos={
                    "joint_1": 0.0,
                    "joint_2": 0.0,
                    "joint_3": 1.5708,
                    "joint_4": 0.0,
                    "joint_5": 1.5708,
                    "joint_6": 0.0,
                    # "rh_r1_joint": 0.04,
                }                         
           ),
            spawn=UsdFileCfg(
                usd_path="/home/jw/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/project_lift/assets/e0509.usd",
                scale=(1.0, 1.0, 1.0),
                
            ),
            actuators={
                "arm": ImplicitActuatorCfg(
                    joint_names_expr=["joint_.*"],
                    stiffness=400.0,
                    damping=80.0,
                ),
            },
            soft_joint_pos_limit_factor=0.9,
        )
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["joint_.*"], scale=0.5, use_default_offset=True
        )
        self.commands.object_pose.body_name = "link_6"

        # Set Cube as object
        # self.scene.object = RigidObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Object",
        #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.055], rot=[1, 0, 0, 0]),
        #     spawn=UsdFileCfg(
        #         usd_path="/home/jw/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/project_lift/assets/spoon.usd",
        #         scale=(1.0, 1.0, 1.0),
        #         rigid_props=RigidBodyPropertiesCfg(
        #             solver_position_iteration_count=16,
        #             solver_velocity_iteration_count=1,
        #             max_angular_velocity=1000.0,
        #             max_linear_velocity=1000.0,
        #             max_depenetration_velocity=5.0,
        #             disable_gravity=False,
        #         ),
        #     ),
        # )

        # object_marker_cfg = FRAME_MARKER_CFG.copy()
        # object_marker_cfg.markers["frame"].scale = (0.07, 0.07, 0.07)
        # object_marker_cfg.prim_path = "/Visuals/ObjectFrame"

        # self.scene.object_frame = FrameTransformerCfg(
        #     prim_path="{ENV_REGEX_NS}/Object/spoon",   # 접시 root prim 경로
        #     debug_vis=True,
        #     visualizer_cfg=object_marker_cfg,
        #     target_frames=[
        #         FrameTransformerCfg.FrameCfg(
        #             prim_path="{ENV_REGEX_NS}/Object/spoon",
        #             name="spoon",
        #             offset=OffsetCfg(),  # identity
        #         ),
        #     ],
        # )


        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/e0509/base_link",
            debug_vis=True,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/e0509/link_6",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.0],
                    ),
                ),
            ],
        )


@configclass
class FrankaCubeLiftEnvCfg_PLAY(FrankaCubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
