# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING
from isaaclab.sensors import FrameTransformer
from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms, quat_apply

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_reached_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "object_pose",
    threshold: float = 0.02,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Termination condition for the object reaching the goal position.

    Args:
        env: The environment.
        command_name: The name of the command that is used to control the object.
        threshold: The threshold for the object to reach the goal position. Defaults to 0.02.
        robot_cfg: The robot configuration. Defaults to SceneEntityCfg("robot").
        object_cfg: The object configuration. Defaults to SceneEntityCfg("object").

    """
    # extract the used quantities (to enable type-hinting)
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)
    # compute the desired position in the world frame
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, des_pos_b)
    # distance of the end-effector to the object: (num_envs,)
    distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)

    # rewarded if the object is lifted above the threshold
    return distance < threshold


def distance_limit(
    env: ManagerBasedRLEnv,
    threshold: float,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    
    # extract the used quantities (to enable type-hinting)
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    # compute the desired position in the world frame
    ee_pos = ee_frame.data.target_pos_source[..., 0, :]   # shape: [N, 3]

    # distance
    distance = torch.norm(ee_pos, dim=1)

    return distance >= threshold


def ee_reached_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "object_pose",
    threshold: float = 0.02,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    # Robot base transform
    robot: RigidObject = env.scene[robot_cfg.name]
    robot_pos_w = robot.data.root_pos_w[:, :3]
    robot_quat_w = robot.data.root_quat_w[:, :4]

    # Commanded pose (body frame → target)
    command = env.command_manager.get_command(command_name)
    target_pos_b = command[:, :3]  # (N,3)

    # Convert target position to world frame
    target_pos_w, _ = combine_frame_transforms(robot_pos_w, robot_quat_w, target_pos_b)

    # EE position in world frame
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]  # (N,3)

    # Distance between EE and goal
    dist = torch.norm(target_pos_w - ee_pos_w, dim=1)

    return dist < threshold  # Boolean tensor


def ee_out_of_safe_box(
    env: ManagerBasedRLEnv,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """
    EE가 특정 안전 영역 박스를 벗어나면 episode 종료.

    안전 영역:
        -0.05 < x < 0.6
        -0.275 < y < 0.6
        0 < z  (테이블 아래로 내려가면 종료)
    """

    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    
    # EE world position
    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]   # (N,3)
    x = ee_pos_w[:, 0]
    y = ee_pos_w[:, 1]
    z = ee_pos_w[:, 2]

    # 조건
    out_x = (x < -0.05) | (x > 0.6)
    out_y = (y < -0.275) | (y > 0.6)
    out_z = (z < 0.0)

    # 하나라도 만족하면 종료
    done = out_x | out_y | out_z
    return done





