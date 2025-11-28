# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Sequence

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
from isaaclab.utils.math import combine_frame_transforms
from .common import TABLEWARE_OBJECT_CFGS

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_is_lifted(
    env: ManagerBasedRLEnv,
    minimal_height: float,
    object_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Reward the agent for lifting the given object above the minimal height."""
    obj: RigidObject = env.scene[object_cfg.name]
    return torch.where(obj.data.root_pos_w[:, 2] > minimal_height, 1.0, 0.0)


# 필요하면 나중에 단일 object용 reach / goal 리워드 다시 쓸 수 있게 남겨둠
# (지금은 사용 안 하니까 주석 상태로 둬도 됨)

# def object_ee_distance(
#     env: ManagerBasedRLEnv,
#     std: float,
#     object_cfg: SceneEntityCfg,
#     ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
# ) -> torch.Tensor:
#     """Reward the agent for reaching the given object using tanh-kernel."""
#     obj: RigidObject = env.scene[object_cfg.name]
#     ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
#     cube_pos_w = obj.data.root_pos_w
#     ee_w = ee_frame.data.target_pos_w[..., 0, :]
#     dist = torch.norm(cube_pos_w - ee_w, dim=1)
#     return 1 - torch.tanh(dist / std)
#
#
# def object_goal_distance(
#     env: ManagerBasedRLEnv,
#     std: float,
#     minimal_height: float,
#     command_name: str,
#     robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
#     object_cfg: SceneEntityCfg,
# ) -> torch.Tensor:
#     """Reward the agent for tracking the goal pose of a given object."""
#     robot: RigidObject = env.scene[robot_cfg.name]
#     obj: RigidObject = env.scene[object_cfg.name]
#     command = env.command_manager.get_command(command_name)
#     des_pos_b = command[:, :3]
#     des_pos_w, _ = combine_frame_transforms(
#         robot.data.root_pos_w, robot.data.root_quat_w, des_pos_b
#     )
#     dist = torch.norm(des_pos_w - obj.data.root_pos_w, dim=1)
#     return (obj.data.root_pos_w[:, 2] > minimal_height) * (1 - torch.tanh(dist / std))


def tableware_ee_distance_min(
    env: ManagerBasedRLEnv,
    std: float,
    object_cfgs: Sequence[SceneEntityCfg] | None = None,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """접시/숟가락/포크 중 EE와 가장 가까운 물체 기준 거리 리워드."""
    if object_cfgs is None:
        object_cfgs = TABLEWARE_OBJECT_CFGS

    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_w = ee_frame.data.target_pos_w[..., 0, :]  # (num_env, 3)

    dists = []
    for cfg in object_cfgs:
        obj: RigidObject = env.scene[cfg.name]
        pos = obj.data.root_pos_w[:, :3]
        d = torch.norm(pos - ee_w, dim=1)  # (num_env,)
        dists.append(d)

    dist_min = torch.stack(dists, dim=-1).min(dim=-1).values  # (num_env,)
    return 1 - torch.tanh(dist_min / std)
