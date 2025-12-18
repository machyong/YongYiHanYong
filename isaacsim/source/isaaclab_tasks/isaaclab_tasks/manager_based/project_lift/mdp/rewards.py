# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from __future__ import annotations
import torch
import torch.nn.functional as F

from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
from isaaclab.utils.math import combine_frame_transforms, matrix_from_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ee_distance_gaussian(
    env: ManagerBasedRLEnv,
    std_xy: float,
    std_z: float,
    # object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward agent: first reward z-lift, then reward xy positioning."""
    # object: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    # cube_pos_w = object.data.root_pos_w

    robot = env.scene["robot"]
    robot_rot = matrix_from_quat(robot.data.root_quat_w)
    robot_pos_w = robot.data.root_pos_w[:, :3]    

    cmd = env.command_manager.get_command("object_pose")
    target_pos_b = cmd[..., :3]  # (N, 3)

    target_pos_w = (robot_rot @ target_pos_b.unsqueeze(-1)).squeeze(-1) + robot_pos_w

    ee_pos_w = ee_frame.data.target_pos_w[..., 0, :]
    delta = target_pos_w - ee_pos_w
    dx, dy, dz = delta[:, 0], delta[:, 1], delta[:, 2]
    # XY 평면 Gaussian 보상
    reward_xy = torch.exp(-(dx**2 + dy**2) / (2 * std_xy**2))
    # Z Gaussian 보상 (XY 가까워야 의미 있음)
    reward_z = torch.exp(-(dz**2) / (2 * std_z**2))
    # 최종 보상
    reward = reward_xy * reward_z
    return reward

def ee_distance_tanh(
    env: ManagerBasedRLEnv,
    std_xy: float,
    std_z: float,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    robot = env.scene["robot"]
    robot_rot = matrix_from_quat(robot.data.root_quat_w)
    robot_pos_w = robot.data.root_pos_w[:, :3]

    cmd = env.command_manager.get_command("object_pose")
    target_pos_b = cmd[..., :3]  # (N, 3)
    target_pos_w = (robot_rot @ target_pos_b.unsqueeze(-1)).squeeze(-1) + robot_pos_w

    ee_pos_w = ee_frame.data.target_pos_w[..., 0, :]
    delta = target_pos_w - ee_pos_w
    dx, dy, dz = delta[:, 0], delta[:, 1], delta[:, 2]

    d_xy = torch.sqrt(dx**2 + dy**2 + 1e-8)   # (N,)
    d_z  = torch.abs(dz)                      # (N,)

    reward_xy = 1.0 - torch.tanh(d_xy / (std_xy + 1e-8))  # 0~1
    reward_z  = 1.0 - torch.tanh(d_z  / (std_z  + 1e-8))  # 0~1

    reward = reward_xy * reward_z
    return reward


def spoon_gripper_perpendicular(  # 숟가락과 그리퍼 수직 정도 계산 함수
    env: ManagerBasedRLEnv,  # 환경
    # object_cfg: SceneEntityCfg = SceneEntityCfg("object"),  # 물체(숟가락) 설정
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame")  # 엔드 이펙터(그리퍼) 설정
) -> torch.Tensor:  # 반환 타입: 토치 텐서
    """숟가락의 x축과 그리퍼 close 축(gripper x축)이 수직일 때 보상을 준다.
    두 벡터의 외적을 이용하여 수직 정도를 계산한다.
    - 외적의 크기가 크면 두 축이 수직 (보상 증가)
    - 외적의 크기가 작으면 두 축이 평행 (보상 감소)
    """
    # 환경에서 물체 객체 가져오기
    # object: RigidObject = env.scene[object_cfg.name]  # 씬에서 물체 가져오기
    # 환경에서 엔드 이펙터 객체 가져오기
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]  # 씬에서 엔드 이펙터 가져오기
    # 숟가락의 쿼터니언을 회전 행렬로 변환하여 x축 벡터 추출
    # spoon_quat = object.data.root_quat_w  # 숟가락 쿼터니언 (환경 수, 4)
    # spoon_rot_matrix = matrix_from_quat(spoon_quat)  # 회전 행렬로 변환 (환경 수, 3, 3)
    # spoon_x_world = spoon_rot_matrix[:, 0, :]  # 회전 행렬의 첫 번째 열 (x축)
    # 그리퍼의 쿼터니언을 회전 행렬로 변환하여 x축 벡터 추출

    robot = env.scene["robot"]
    robot_rot = matrix_from_quat(robot.data.root_quat_w)
    robot_pos_w = robot.data.root_pos_w[:, :3]    

    cmd = env.command_manager.get_command("object_pose")
    target_pos_b = cmd[..., :3]  # (N, 3)

    target_pos_w = (robot_rot @ target_pos_b.unsqueeze(-1)).squeeze(-1) + robot_pos_w

    x = target_pos_w[..., 0]

    target_x_vec_world = torch.stack(
        [x, torch.zeros_like(x), torch.zeros_like(x)], dim=-1
    )                                            # (N, 3)

    # 방향 벡터로 사용하려면 정규화
    target_x_dir_world = F.normalize(target_x_vec_world, dim=-1)

    gripper_quat = ee_frame.data.target_quat_w  # 그리퍼 쿼터니언 (환경 수, 타겟 수, 4)
    gripper_quat = gripper_quat[:, 0, :]  # 첫 번째 타겟만 선택 (환경 수, 4)
    gripper_rot_matrix = matrix_from_quat(gripper_quat)  # 회전 행렬로 변환 (환경 수, 3, 3)
    gripper_x_world = gripper_rot_matrix[:, 0, :]  # 회전 행렬의 첫 번째 열 (x축)
    # 숟가락 x축을 정규화
    # 그리퍼 x축을 정규화
    gripper_x_world = torch.nn.functional.normalize(gripper_x_world, dim=-1)  # 그리퍼 벡터 정규화
    # 두 벡터의 내적 계산 (코사인 유사도)
    dot = (target_x_dir_world * gripper_x_world).sum(dim=-1)  # 내적 계산
    # 직교 정도를 보상으로 변환 (수직일수록 보상 증가)
    reward = 1.0 - torch.abs(dot)  # 보상 = 1 - |내적| (0~1, 1이면 직교)
    # 계산된 보상 반환
    return reward  # 보상 반환


def gripper_horizontal_spoon(  # 그리퍼 수평성 보상 함수
    env: ManagerBasedRLEnv,  # 환경
    std_xy: float,
    std_z: float,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),  # 엔드 이펙터 설정
) -> torch.Tensor:  # 반환 타입: 토치 텐서
    """그리퍼가 수평(z축이 아래를 향함)일 때 보상을 준다.
    그리퍼의 z축이 -z 방향(아래)을 향할수록 보상이 증가한다.
    - z축이 (0, 0, -1) 방향이면: 보상 = 1.0 (완벽한 수평)
    - z축이 (0, 0, 1) 방향이면: 보상 = 0.0 (역방향)
    """
    # 환경에서 엔드 이펙터 객체 가져오기
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]  # 씬에서 엔드 이펙터 가져오기
    # 그리퍼의 z축 방향 벡터 (아래쪽을 향해야 함)
    # 쿼터니언을 회전 행렬로 변환하여 z축 추출
    # 쿼터니언을 회전 행렬로 변환

    robot = env.scene["robot"]
    robot_rot = matrix_from_quat(robot.data.root_quat_w)
    robot_pos_w = robot.data.root_pos_w[:, :3]    

    cmd = env.command_manager.get_command("object_pose")
    target_pos_b = cmd[..., :3]  # (N, 3)

    target_pos_w = (robot_rot @ target_pos_b.unsqueeze(-1)).squeeze(-1) + robot_pos_w

    ee_pos_w = ee_frame.data.target_pos_w[..., 0, :]
    delta = target_pos_w - ee_pos_w
    dx, dy, dz = delta[:, 0], delta[:, 1], delta[:, 2]

    gripper_quat = ee_frame.data.target_quat_w  # 그리퍼 쿼터니언 (환경 수, 타겟 수, 4)
    gripper_quat = gripper_quat[:, 0, :]  # 첫 번째 타겟만 선택 (환경 수, 4)
    ee_rot_matrix = matrix_from_quat(gripper_quat)  # 회전 행렬로 변환 (환경 수, 3, 3)
    # 회전 행렬의 세 번째 열을 z축으로 추출
    gripper_z_axis = ee_rot_matrix[:, 2, :]  # 회전 행렬의 z축 벡터 추출
    gripper_x_axis = ee_rot_matrix[:, 0, :] 
    gripper_y_axis = ee_rot_matrix[:, 1, :] 
    # 목표 방향 설정: (0, 0, -1) - 아래쪽을 향함
    target_x_axis = torch.tensor([1.0, 0.0, 0.0], device=gripper_x_axis.device)
    target_y_axis = torch.tensor([0.0, 1.0, 0.0], device=gripper_y_axis.device)
    target_z_axis = torch.tensor([0.0, 0.0, -1.0], device=gripper_z_axis.device)  # 목표 벡터 정의
    # 그리퍼 z축을 정규화 (이미 단위 벡터이지만 안전하게)
    gripper_z_axis = torch.nn.functional.normalize(gripper_z_axis, dim=-1)  # 벡터 정규화
    # 두 벡터의 내적 계산 (-1 ~ 1 범위)
    # 내적이 1에 가까우면 같은 방향 (수평)
    # 내적이 -1에 가까우면 반대 방향

    reward_xy = torch.exp(-(dx**2 + dy**2) / (2 * std_xy**2))

    reward_z = torch.exp(-(dz**2) / (2 * std_z**2))

    dot_product = (gripper_z_axis * target_z_axis).sum(dim=-1)  # 내적 계산
    # 내적을 보상으로 변환: -1 ~ 1을 0 ~ 1 범위로
    # dot = 1 (완벽한 수평) → reward = 1.0
    # dot = 0 (수직) → reward = 0.0
    # dot = -1 (역방향) → reward = 0.0
    reward_h_z = torch.clamp(dot_product, min=0.0) # 내적을 0 이상으로 클램핑하여 보상 계산
    # 계산된 보상 반환

    dot_product_x = (gripper_z_axis * target_x_axis).sum(dim=-1)
    dot_product_y = (gripper_z_axis * target_y_axis).sum(dim=-1)

    reward_h_x = 1 - dot_product_x.abs()
    reward_h_y = 1 - dot_product_y.abs()
    reward = reward_h_z * reward_h_x * reward_h_y 

    return reward_xy * reward_z * reward # 보상 반환

def gripper_horizontal_w(
    env: ManagerBasedRLEnv,
    std_xy: float,
    std_z: float,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """EE 위치 + (EE roll 180도 적용 후) target orientation과 일치 보상.

    - EE roll을 로컬 x축 기준 180도 회전시킨 후 target orientation과 비교
    - 회전 각도가 0일수록 보상 1
    """

    robot = env.scene["robot"]
    robot_rot = matrix_from_quat(robot.data.root_quat_w)
    robot_pos_w = robot.data.root_pos_w[:, :3]    
    # joint_pos = robot.data.joint_pos
    # joint_vel = robot.data.joint_vel
    # print("joint_pos[env0]:", joint_pos[0].detach().cpu().tolist(), flush=True)
    # print("joint_vel[env0]:", joint_vel[0].detach().cpu().tolist(), flush=True)

    # --------------------------------------------------
    # 1) Target position & orientation
    # --------------------------------------------------
    cmd = env.command_manager.get_command("object_pose")
    target_pos_b = cmd[..., :3]       # (N, 3)
    target_quat = cmd[..., 3:7]       # (N, 4)  [w, x, y, z]

    target_rot = matrix_from_quat(target_quat)     # (N, 3, 3)

    # --------------------------------------------------
    # 2) EE position & orientation
    # --------------------------------------------------
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    ee_pos = ee_frame.data.target_pos_source[..., 0, :]    # (N, 3)
    ee_quat = ee_frame.data.target_quat_source[:, 0, :]    # (N, 4)

    ee_rot = matrix_from_quat(ee_quat)  # (N, 3, 3)

    # --------------------------------------------------
    # 3) 위치 보상 (xy, z)
    # --------------------------------------------------
    target_pos_w = (robot_rot @ target_pos_b.unsqueeze(-1)).squeeze(-1) + robot_pos_w
    ee_pos_w = ee_frame.data.target_pos_w[..., 0, :]
    delta = target_pos_w - ee_pos_w
    dx, dy, dz = delta[:, 0], delta[:, 1], delta[:, 2]

    reward_xy = torch.exp(-(dx**2 + dy**2) / (2 * std_xy**2))
    reward_z  = torch.exp(-(dz**2) / (2 * std_z**2))

    # --------------------------------------------------
    # 4) EE roll을 로컬 x축 기준 180도 회전 적용
    #    quaternion 기준 [w,x,y,z] = [0,1,0,0]
    # --------------------------------------------------
    device = ee_rot.device
    dtype  = ee_rot.dtype

    # Rx(pi) 회전행렬 적용 방식 (local frame)
    Rx_pi = torch.tensor(
        [[1.0,  0.0,  0.0],
         [0.0, -1.0,  0.0],
         [0.0,  0.0, -1.0]],
        device=device,
        dtype=dtype,
    ).unsqueeze(0)        # (1,3,3)

    # EE 회전행렬에 roll 180도 적용 (local)
    ee_rot_roll = ee_rot @ Rx_pi       # (N,3,3)

    # --------------------------------------------------
    # 5) orientation 보상 = angle difference
    # --------------------------------------------------
    # relative rotation: R_rel = R_target^T * R_ee_roll
    rot_rel = torch.matmul(target_rot.transpose(1, 2), ee_rot_roll)

    # cos(theta) = (trace(R_rel) - 1) / 2
    trace = rot_rel[:,0,0] + rot_rel[:,1,1] + rot_rel[:,2,2]
    cos_theta = (trace - 1.0) * 0.5
    cos_theta = torch.clamp(cos_theta, -1.0+1e-6, 1.0-1e-6)

    theta = torch.acos(cos_theta)   # (N,) rad

    # Gaussian reward for angle
    sigma_ang = 0.3
    reward_orient = torch.exp(-(theta**2) / (2 * sigma_ang**2))

    # --------------------------------------------------
    # 6) Final reward
    # --------------------------------------------------
    return reward_xy * reward_z * reward_orient

def gripper_horizontal(
    env: ManagerBasedRLEnv,
    std_xy: float,
    std_z: float,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """EE 위치 + (EE roll 180도 적용 후) target orientation과 일치 보상.

    - EE roll을 로컬 x축 기준 180도 회전시킨 후 target orientation과 비교
    - 회전 각도가 0일수록 보상 1
    """

    # --------------------------------------------------
    # 1) Target position & orientation
    # --------------------------------------------------
    cmd = env.command_manager.get_command("object_pose")
    target_pos_b = cmd[..., :3]       # (N, 3)
    target_quat = cmd[..., 3:7]       # (N, 4)  [w, x, y, z]

    target_rot = matrix_from_quat(target_quat)     # (N, 3, 3)
    
    # --------------------------------------------------
    # 2) EE position & orientation
    # --------------------------------------------------
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    ee_pos = ee_frame.data.target_pos_source[..., 0, :]    # (N, 3)
    ee_quat = ee_frame.data.target_quat_source[:, 0, :]    # (N, 4)

    ee_rot = matrix_from_quat(ee_quat)  # (N, 3, 3)

    # --------------------------------------------------
    # 3) 위치 보상 (xy, z)
    # --------------------------------------------------
    delta = target_pos_b - ee_pos
    dx, dy, dz = delta[:, 0], delta[:, 1], delta[:, 2]

    reward_xy = torch.exp(-(dx**2 + dy**2) / (2 * std_xy**2))
    reward_z  = torch.exp(-(dz**2) / (2 * std_z**2))

    # --------------------------------------------------
    # 4) EE roll을 로컬 x축 기준 180도 회전 적용
    #    quaternion 기준 [w,x,y,z] = [0,1,0,0]
    # --------------------------------------------------
    device = ee_rot.device
    dtype  = ee_rot.dtype

    # Rx(pi) 회전행렬 적용 방식 (local frame)
    Rx_pi = torch.tensor(
        [[1.0,  0.0,  0.0],
         [0.0, -1.0,  0.0],
         [0.0,  0.0, -1.0]],
        device=device,
        dtype=dtype,
    ).unsqueeze(0)        # (1,3,3)

    # EE 회전행렬에 roll 180도 적용 (local)
    ee_rot_roll = ee_rot @ Rx_pi       # (N,3,3)

    # --------------------------------------------------
    # 5) orientation 보상 = angle difference
    # --------------------------------------------------
    # relative rotation: R_rel = R_target^T * R_ee_roll
    rot_rel = torch.matmul(target_rot.transpose(1, 2), ee_rot_roll)

    # cos(theta) = (trace(R_rel) - 1) / 2
    trace = rot_rel[:,0,0] + rot_rel[:,1,1] + rot_rel[:,2,2]
    cos_theta = (trace - 1.0) * 0.5
    cos_theta = torch.clamp(cos_theta, -1.0+1e-6, 1.0-1e-6)

    theta = torch.acos(cos_theta)   # (N,) rad

    # Gaussian reward for angle
    sigma_ang = 0.3
    reward_orient = torch.exp(-(theta**2) / (2 * sigma_ang**2))

    # --------------------------------------------------
    # 6) Final reward
    # --------------------------------------------------
    return reward_xy * reward_z * reward_orient



def gripper_reach_horizontal(
    env: ManagerBasedRLEnv,
    std_xy: float,
    std_z: float,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """EE 위치 + (EE roll 180도 적용 후) target orientation과 일치 보상."""

    # 1) Target pose
    cmd = env.command_manager.get_command("object_pose")
    target_pos_b = cmd[..., :3]       # (N, 3)
    target_quat = cmd[..., 3:7]       # (N, 4)
    target_rot = matrix_from_quat(target_quat)     # (N, 3, 3)

    # 2) EE pose
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_pos = ee_frame.data.target_pos_source[..., 0, :]    # (N, 3)
    ee_quat = ee_frame.data.target_quat_source[:, 0, :]    # (N, 4)
    ee_rot = matrix_from_quat(ee_quat)  # (N, 3, 3)

    # 3) 위치 보상 (xy, z)
    delta = target_pos_b - ee_pos
    dx, dy, dz = delta[:, 0], delta[:, 1], delta[:, 2]

    # --- xy 거리 ---
    dist_xy = torch.sqrt(dx**2 + dy**2 + 1e-8)

    # (1) 멀리서: 선형 보상
    max_dist = 1.1 
    reward_xy_linear = 1.0 - torch.clamp(dist_xy / max_dist, 0.0, 1.0)

    # (2) 가까이(<=0.1): 가우시안 보상
    near_radius = 0.08
    reward_xy_gauss = torch.exp(-(dist_xy**2) / (2 * std_xy**2))  # std_xy=0.1 추천

    # piecewise 결합
    reward_xy = torch.where(dist_xy > near_radius,
                            reward_xy_linear,
                            reward_xy_gauss)

    # z는 그대로 가우시안
    reward_z = torch.exp(-(dz**2) / (2 * std_z**2))

    # 4) EE roll 180도 적용 (orientation 보상은 기존 그대로)
    device = ee_rot.device
    dtype  = ee_rot.dtype

    Rx_pi = torch.tensor(
        [[1.0,  0.0,  0.0],
         [0.0, -1.0,  0.0],
         [0.0,  0.0, -1.0]],
        device=device,
        dtype=dtype,
    ).unsqueeze(0)

    ee_rot_roll = ee_rot @ Rx_pi

    rot_rel = torch.matmul(target_rot.transpose(1, 2), ee_rot_roll)
    trace = rot_rel[:,0,0] + rot_rel[:,1,1] + rot_rel[:,2,2]
    cos_theta = (trace - 1.0) * 0.5
    cos_theta = torch.clamp(cos_theta, -1.0+1e-6, 1.0-1e-6)
    theta = torch.acos(cos_theta)

    sigma_ang = 0.3
    reward_orient = torch.exp(-(theta**2) / (2 * sigma_ang**2))

    return reward_xy * reward_z * reward_orient

def gripper_horizontal_field(
    env: ManagerBasedRLEnv,
    std_z: float,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    k_field: float = 5.0,
    sigma_orient: float = 0.3,
) -> torch.Tensor:
    """
    Potential Field 보상 (이전 위치 없이)
    
    - EE가 목표 위치 근처에 있을수록 reward 증가
    - 방향성: EE 위치 벡터와 목표 벡터의 alignment(dot) 이용
    - 가까워지면 orientation reward 적용
    """

    # 1) Target position & orientation
    cmd = env.command_manager.get_command("object_pose")
    target_pos_b = cmd[..., :3]       # (N, 3)
    target_quat = cmd[..., 3:7]       # (N, 4)
    target_rot = matrix_from_quat(target_quat)

    # 2) EE position & orientation
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_pos = ee_frame.data.target_pos_source[..., 0, :]
    ee_quat = ee_frame.data.target_quat_source[:, 0, :]
    ee_rot = matrix_from_quat(ee_quat)

    # 3) Potential Field 위치 보상 (이전 위치 없이 dot 사용)
    delta = target_pos_b - ee_pos       # 목표 벡터
    dist = torch.norm(delta, dim=1)
    unit_delta = delta / (dist.unsqueeze(1) + 1e-6)

    # EE 위치 벡터(원점 기준) 정규화
    ee_vec = ee_pos / (torch.norm(ee_pos, dim=1).unsqueeze(1) + 1e-6)

    # 목표 방향과 EE 위치 alignment
    reward_field = torch.sum(unit_delta * ee_vec, dim=1) * k_field
    reward_field = torch.clamp(reward_field, min=0.0)

    # 4) Z 위치 보상
    dz = delta[:, 2]
    reward_z = torch.exp(-(dz**2) / (2 * std_z**2))

    # 5) Orientation reward
    device = ee_rot.device
    dtype  = ee_rot.dtype
    Rx_pi = torch.tensor(
        [[1.0, 0.0, 0.0],
         [0.0,-1.0, 0.0],
         [0.0, 0.0,-1.0]],
        device=device,
        dtype=dtype
    ).unsqueeze(0)
    ee_rot_roll = ee_rot @ Rx_pi

    rot_rel = torch.matmul(target_rot.transpose(1, 2), ee_rot_roll)
    trace = rot_rel[:,0,0] + rot_rel[:,1,1] + rot_rel[:,2,2]
    cos_theta = torch.clamp((trace - 1.0)*0.5, -1.0+1e-6, 1.0-1e-6)
    theta = torch.acos(cos_theta)
    reward_orient = torch.exp(-(theta**2) / (2 * sigma_orient**2))

    # 6) 최종 reward: 가까워지면 orientation reward 포함
    dist_threshold = 0.05
    orient_mask = (dist < dist_threshold).float()
    reward = reward_field * reward_z + orient_mask * reward_orient

    return reward

def gripper_h(
    env: ManagerBasedRLEnv,
    std_xy: float,
    std_z: float,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """EE 위치 + (EE roll 180도 적용 후) target orientation과 일치 보상.

    - EE roll을 로컬 x축 기준 180도 회전시킨 후 target orientation과 비교
    - 회전 각도가 0일수록 보상 1
    """

    robot = env.scene["robot"]
    joint_pos = robot.data.joint_pos
    joint_vel = robot.data.joint_vel
    print("joint_pos[env0]:", joint_pos[0].detach().cpu().tolist(), flush=True)
    print("joint_vel[env0]:", joint_vel[0].detach().cpu().tolist(), flush=True)

    # --------------------------------------------------
    # 1) Target position & orientation
    # --------------------------------------------------
    cmd = env.command_manager.get_command("object_pose")
    target_pos_b = cmd[..., :3]       # (N, 3)
    target_quat = cmd[..., 3:7]       # (N, 4)  [w, x, y, z]

    target_rot = matrix_from_quat(target_quat)     # (N, 3, 3)

    # --------------------------------------------------
    # 2) EE position & orientation
    # --------------------------------------------------
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    ee_pos = ee_frame.data.target_pos_source[..., 0, :]    # (N, 3)
    ee_quat = ee_frame.data.target_quat_source[:, 0, :]    # (N, 4)

    ee_rot = matrix_from_quat(ee_quat)  # (N, 3, 3)

    # --------------------------------------------------
    # 3) 위치 보상 (xy, z)
    # --------------------------------------------------
    delta = target_pos_b - ee_pos
    dx, dy, dz = delta[:, 0], delta[:, 1], delta[:, 2]

    reward_xy = torch.exp(-(dx**2 + dy**2) / (2 * std_xy**2))
    reward_z  = torch.exp(-(dz**2) / (2 * std_z**2))

    # --------------------------------------------------
    # 4) EE roll을 로컬 x축 기준 180도 회전 적용
    #    quaternion 기준 [w,x,y,z] = [0,1,0,0]
    # --------------------------------------------------
    device = ee_rot.device
    dtype  = ee_rot.dtype

    # Rx(pi) 회전행렬 적용 방식 (local frame)
    Rx_pi = torch.tensor(
        [[1.0,  0.0,  0.0],
         [0.0, -1.0,  0.0],
         [0.0,  0.0, -1.0]],
        device=device,
        dtype=dtype,
    ).unsqueeze(0)        # (1,3,3)

    # EE 회전행렬에 roll 180도 적용 (local)
    ee_rot_roll = ee_rot @ Rx_pi       # (N,3,3)

    # --------------------------------------------------
    # 5) orientation 보상 = angle difference
    # --------------------------------------------------
    # relative rotation: R_rel = R_target^T * R_ee_roll
    rot_rel = torch.matmul(target_rot.transpose(1, 2), ee_rot_roll)

    # cos(theta) = (trace(R_rel) - 1) / 2
    trace = rot_rel[:,0,0] + rot_rel[:,1,1] + rot_rel[:,2,2]
    cos_theta = (trace - 1.0) * 0.5
    cos_theta = torch.clamp(cos_theta, -1.0+1e-6, 1.0-1e-6)

    theta = torch.acos(cos_theta)   # (N,) rad

    # Gaussian reward for angle
    sigma_ang = 0.3
    reward_orient = torch.exp(-(theta**2) / (2 * sigma_ang**2))

    # --------------------------------------------------
    # 6) Final reward
    # --------------------------------------------------
    return reward_xy * reward_z * reward_orient


def object_is_lifted(
    env: ManagerBasedRLEnv, minimal_height: float, object_cfg: SceneEntityCfg = SceneEntityCfg("object")
) -> torch.Tensor:
    """Reward the agent for lifting the object above the minimal height."""
    object: RigidObject = env.scene[object_cfg.name]
    return torch.where(object.data.root_pos_w[:, 2] > minimal_height, 1.0, 0.0)


def object_ee_distance(
    env: ManagerBasedRLEnv,
    std: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward the agent for reaching the object using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    object: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    # Target object position: (num_envs, 3)
    cube_pos_w = object.data.root_pos_w
    # End-effector position: (num_envs, 3)
    ee_w = ee_frame.data.target_pos_w[..., 0, :]
    # Distance of the end-effector to the object: (num_envs,)
    object_ee_distance = torch.norm(cube_pos_w - ee_w, dim=1)

    return 1 - torch.tanh(object_ee_distance / std)


def object_goal_distance(
    env: ManagerBasedRLEnv,
    std: float,
    minimal_height: float,
    command_name: str,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object"),
) -> torch.Tensor:
    """Reward the agent for tracking the goal pose using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)
    # compute the desired position in the world frame
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, des_pos_b)
    # distance of the end-effector to the object: (num_envs,)
    distance = torch.norm(des_pos_w - object.data.root_pos_w, dim=1)
    # rewarded if the object is lifted above the threshold
    return (object.data.root_pos_w[:, 2] > minimal_height) * (1 - torch.tanh(distance / std))
