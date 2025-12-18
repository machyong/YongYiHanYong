#!/usr/bin/env python3
"""
Doosan E0509 + IsaacLab PPO 정책 Position Controller 제어 노드

추가 기능:
- /move_to_point_rl(StringToString) 서비스 서버 추가
- 요청(req.input) → target_cmd 업데이트
- 목표 10cm 도달하면 → 서비스 응답(output=req.input 그대로)
"""

from __future__ import annotations

import math
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped, PoseStamped

from tf2_ros import Buffer, TransformListener

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
import ast
import os
from ament_index_python.packages import get_package_share_directory
### ADD ###
from interface_pkg.srv import StringToString   # 서비스 인터페이스 추가


# =============================================================================
#  OBS BUILDER
# =============================================================================
class SpoonBussingObsBuilder:
    def __init__(self, device: str = "cpu"):
        self.device = torch.device(device)

        self.default_joint_pos = np.array([0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0], dtype=np.float32)
        self.default_joint_vel = np.zeros(6, dtype=np.float32)
        self.last_action = np.zeros(6, dtype=np.float32)

    @staticmethod
    def _to_np(arr, expected_dim: int, name: str):
        if isinstance(arr, torch.Tensor):
            arr = arr.detach().cpu().numpy()
        arr = np.asarray(arr, dtype=np.float32).reshape(-1)
        if arr.shape[0] != expected_dim:
            raise ValueError(f"{name} 길이 {expected_dim}이어야 하는데 {arr.shape[0]} 입니다.")
        return arr

    def set_last_action(self, action):
        self.last_action[:] = self._to_np(action, 6, "action")

    def build_obs(self, joint_pos, joint_vel, target_cmd):
        q = self._to_np(joint_pos, 6, "joint_pos")
        dq = self._to_np(joint_vel, 6, "joint_vel")

        q_rel = q - self.default_joint_pos
        dq_rel = dq - self.default_joint_vel

        tgt = np.asarray(target_cmd, dtype=np.float32)
        if tgt.shape[0] == 3:
            x, y, z = tgt
            tgt = np.array([x, y, z, 1, 0, 0, 0], dtype=np.float32)

        obs_np = np.concatenate([q_rel, dq_rel, tgt, self.last_action], axis=0)
        return torch.from_numpy(obs_np).unsqueeze(0).to(self.device)


# =============================================================================
#  PPO CONTROL NODE + SERVICE SERVER
# =============================================================================
class PPOPositionControlNode(Node):
    def __init__(self):
        super().__init__("ppo_position_control_node")

        package_path = get_package_share_directory("llm_for_pick_place_voice")
        self.policy_path = os.path.join(package_path, "resource", "spoon_bussing_policy.pt")
        
        self.compute_dt = 0.05  # 20Hz
        self.publish_every_n_steps = 1
        self.action_scale = 0.03
        self.max_delta_deg_per_step = 5.0
        self.max_delta_rad = math.radians(self.max_delta_deg_per_step)
        self.filter_alpha = 0.9

        #target positions
        self.cutlery_up = [0.455, 0.070, 0.350, 1,0,0,0]   # 커틀러리 pick up 위치
        self.dish_up = [0.560, -0.010, 0.350, 1,0,0,0]       # 접시 pick up 위치
        self.cup_up = [0.660, -0.045, 0.350, 1,0,0,0] # 컵 pick up 위치

        limit = math.radians(180)
        self.joint_min = np.array([-limit] * 6, dtype=np.float32)
        self.joint_max = np.array([limit] * 6, dtype=np.float32)

        # ===== STOP flag =====
        self.stop_motion = False  

        # ===== Load Policy =====
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        try:
            self.policy = torch.jit.load(self.policy_path, map_location=device)
            self.policy.eval()
        except Exception as e:
            self.get_logger().error(f"Policy load failed: {e}")
            self.policy = None

        # ===== OBS Builder =====
        self.obs_builder = SpoonBussingObsBuilder(device=device)

        # ===== Robot State =====
        self.joint_pos = np.zeros(6, dtype=np.float32)
        self.joint_vel = np.zeros(6, dtype=np.float32)
        self.state_ready = False

        # 현재 제어 목표 - 기본값 (서비스 요청 전에는 None)
        self.target_cmd = None
        self.filtered_target = None
        self.step_count = 0
        # 서비스 요청 전에는 제어 비활성화
        self.active = False

        # ===== TF Listener =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ===== ROS I/O =====
        self.create_subscription(JointState, "/dsr01/joint_states", self.joint_state_cb, 10)

        self.pos_pub = self.create_publisher(
            JointTrajectory,
            "/dsr01/dsr_moveit_controller/joint_trajectory",
            10
        )

        # 20Hz timer
        self.create_timer(self.compute_dt, self.compute_loop)

        ### ADD ###
        # --------------------------
        # move_to_point_rl 서비스 서버
        # --------------------------
        self.current_service_request = None   # 응답을 보낼 때 사용
        self.srv = self.create_service(
            StringToString,
            "/move_to_box_rl",
            self.move_to_point_rl_callback
        )

        self.get_logger().info("PPOPositionControlNode started with move_to_point_rl service.")

    # =============================================================================
    # SERVICE CALLBACK
    # =============================================================================
    ### ADD ###
    def move_to_point_rl_callback(self, request, response):
        """
        1) 요청된 문자열을 내부 target_cmd로 변환
        2) dist < 0.1 될 때까지 compute_loop에서 제어
        3) 조건 만족 시 응답(output=request.input)
        """
        try:
            # 좌표 문자열 예: "0.3,0.2,0.4"
            data = ast.literal_eval(request.input)  # 입력 문자열을 dict로 변환

            center = data["center"]      # 물체 중심 좌표
            angle = data["angle_deg"]    # 물체 각도
            cls = data["class"]          # 물체 클래스명

            self.get_logger().info(f"📌 center      = {center}")
            self.get_logger().info(f"📌 angle_deg   = {angle}")
            self.get_logger().info(f"📌 class       = {cls}, type = {type(cls)}")

            # 1) pick 동작 시퀀스: x,y,z=300mm로 이동 → z=인식된 z값으로 이동 → 그리퍼 동작 → z=300mm로 복귀 → 수거함 위치 이동
            x, y, z = center[0], center[1], center[2]
            # target_cmd이 None이면 새로 생성, 아니면 값만 변경

            if cls == "cutlery":
                    self.target_cmd = np.array(self.cutlery_up, dtype=np.float32)
            elif cls == "dish":
                    self.target_cmd = np.array(self.dish_up, dtype=np.float32)
            elif cls == "cup":

                    self.target_cmd = np.array(self.cup_up, dtype=np.float32)
            else:
                self.get_logger().error(f"Unknown class: {cls}")
                response.output = "ERROR_UNKNOWN_CLASS"
                return response
            self.filtered_target = self.joint_pos.copy()  # ← 이 줄 추가!
            self.stop_motion = False
            self.active = True  # 서비스 요청이 오면 제어 활성화

            self.get_logger().info(f"[RL SERVICE] New Target XYZ: {[x, y, z]}")

            # 응답을 compute_loop 종료 시점에 보내기 위해 저장
            self.current_service_request = (request, response)

            return response  # compute_loop 에서 최종 output 채워서 보내도록 설계됨

        except Exception as e:
            self.get_logger().error(f"Service parse error: {e}")
            response.output = "ERROR"
            return response

    # =============================================================================
    # CALLBACKS
    # =============================================================================
    def joint_state_cb(self, msg):
        self.joint_pos = np.array(msg.position[:6], dtype=np.float32)
        self.joint_vel = np.array(msg.velocity[:6], dtype=np.float32)

        if not self.state_ready:
            self.filtered_target = self.joint_pos.copy()
            self.state_ready = True
            self.joint_pos = [0,0,1.5708,0,1.5708,0]  # 초기 위치로 강제 설정

    # =============================================================================
    # MAIN LOOP (20 Hz)
    # =============================================================================
    def compute_loop(self):
        if not self.state_ready or self.policy is None:
            return

        # 서비스 요청이 오기 전에는 제어하지 않음
        if not self.active or self.target_cmd is None:
            return

        if self.stop_motion:
            return

        self.step_count += 1

        # ---------------------------------------------
        # PPO POLICY INFERENCE
        # ---------------------------------------------
        obs = self.obs_builder.build_obs(self.joint_pos, self.joint_vel, self.target_cmd)

        with torch.no_grad():
            action = self.policy(obs)[0].cpu().numpy()

        self.obs_builder.set_last_action(action)

        joint_target_raw = self.joint_pos + self.action_scale * action
        delta = joint_target_raw - self.joint_pos
        delta_clamped = np.clip(delta, -self.max_delta_rad, self.max_delta_rad)
        joint_target_safe = self.joint_pos + delta_clamped

        # ---------------------------------------------
        # TF → EE 좌표 얻기 + 거리 계산
        # ---------------------------------------------
        try:
            tf_trans = self.tf_buffer.lookup_transform("base_link", "link_6", Time())
            ee_x = tf_trans.transform.translation.x
            ee_y = tf_trans.transform.translation.y

            tgt_x, tgt_y = self.target_cmd[0], self.target_cmd[1]

            dx = ee_x - tgt_x
            dy = ee_y - tgt_y

            dist = math.sqrt(dx*dx + dy*dy)

            # ⭐ 목표 도달 조건
            if dist < 0.05:
                self.get_logger().info(f"🎉 REACHED target: dist={dist:.3f}m → STOP")

                self.stop_motion = True
                self.active = False  # 목표 도달 시 제어 비활성화

                ### ADD ###
                # 서비스 응답 처리
                if self.current_service_request is not None:
                    req, resp = self.current_service_request
                    resp.output = req.input   # 요청값 그대로 반환
                    self.current_service_request = None
                    return resp  # 응답 전송

                return

        except Exception:
            pass

        # ---------------------------------------------
        # filtering + clipping
        # ---------------------------------------------
        if self.filtered_target is None:
            self.filtered_target = joint_target_safe.copy()
        else:
            self.filtered_target = (
                self.filter_alpha * self.filtered_target +
                (1 - self.filter_alpha) * joint_target_safe
            )

        joint_target_final = np.clip(self.filtered_target, self.joint_min, self.joint_max)

        # ---------------------------------------------
        # Publish
        # ---------------------------------------------
        if self.step_count % self.publish_every_n_steps == 0:
            msg = JointTrajectory()
            msg.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]

            point = JointTrajectoryPoint()
            point.positions = joint_target_final.tolist()
            point.time_from_start = Duration(seconds=0.1).to_msg()

            msg.points.append(point)

            self.pos_pub.publish(msg)


# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PPOPositionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
