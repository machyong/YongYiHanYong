# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import subtract_frame_transforms
from .common import TABLEWARE_OBJECT_CFGS

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# def object_position_in_robot_root_frame(
#     env: ManagerBasedRLEnv,
#     robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
#     object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
# ) -> torch.Tensor:
#     """The position of the object in the robot's root frame."""
#     robot: RigidObject = env.scene[robot_cfg.name]
#     object: RigidObject = env.scene[object_cfg.name]
#     object_pos_w = object.data.root_pos_w[:, :3]
#     object_pos_b, _ = subtract_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, object_pos_w)
#     return object_pos_b

def tableware_positions_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfgs: list[SceneEntityCfg] | None = None,
) -> torch.Tensor:
    """dish, spoon, fork 3개 위치를 한 번에 반환 (num_env, 9)."""
    if object_cfgs is None:
        object_cfgs = TABLEWARE_OBJECT_CFGS

    robot: RigidObject = env.scene[robot_cfg.name]
    pos_list = []

    for cfg in object_cfgs:
        obj: RigidObject = env.scene[cfg.name]
        obj_pos_w = obj.data.root_pos_w[:, :3]
        obj_pos_b, _ = subtract_frame_transforms(
            robot.data.root_pos_w, robot.data.root_quat_w, obj_pos_w
        )
        pos_list.append(obj_pos_b)

    return torch.cat(pos_list, dim=-1)  # (num_env, 3 * len(object_cfgs))
