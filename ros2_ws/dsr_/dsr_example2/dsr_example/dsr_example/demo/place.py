#!/usr/bin/env python3
# Python 3 인터프리터로 실행되는 스크립트임을 명시
import sys  # 시스템 관련 기능을 사용하기 위한 모듈

###############################################################
# 1) dsr_description2 패키지가 있는 상위 경로를 등록해야 import 가능
#    - dsr_description2 등 하위 모듈을 import하기 위해 sys.path에 경로 추가
###############################################################
sys.path.append("/home/up/ros2_ws/src/doosan-robot2")

import rclpy  # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS 2 노드 클래스
from interface_pkg.srv import StringToString  # 커스텀 서비스 타입
import DR_init  # Doosan 로봇 초기화 관련 모듈
from std_msgs.msg import String  # 표준 메시지 타입 (사용 여부에 따라 다름)
import ast  # 문자열을 안전하게 파싱하기 위한 모듈
import time  # 시간 관련 기능을 사용하기 위한 모듈

from dsr_description2.gripper.gripper_controller import GripperController  # 그리퍼 제어 클래스

###############################################################
# 2) GripperController import
#    - 그리퍼 제어를 위한 클래스 import (실패 시 예외 처리 가능)
###############################################################
# (현재는 바로 import, 예외처리 주석 처리됨)


ROBOT_ID = "dsr01"  # 로봇 네임스페이스 및 ID
ROBOT_MODEL = "e0509"  # 로봇 모델명
VELOCITY, ACC = 150, 50  # 기본 속도 및 가속도 설정



# ======================================================================
#                        Pick & Place Service Class
# ======================================================================
class PickPlaceService(Node):
    """
    ROS 2 서비스 노드 클래스
    - 로봇의 pick & place 동작을 서비스로 제공
    - 그리퍼 제어, 위치 이동, 서비스 콜백 등 포함
    """

    def pick_and_place_sequence(self, target_pos, object_name):
        """
        실제 pick & place 동작 시퀀스 함수
        - 접근 위치로 이동
        - 그리퍼 열기
        - 하강
        - 그리퍼로 물체 잡기
        """
        try:
            safe_offset = 150.0  # 안전 오프셋 (Z축)
            vel = 150  # 이동 속도
            acc = 50   # 이동 가속도

            # 1) 접근 위치 계산 및 이동
            approach = target_pos.copy()
            approach[2] += safe_offset  # Z축으로 안전거리 확보

            self.get_logger().info("🚀 Step1: Approach")
            self.movel(approach, vel, acc, ref=self.DR_BASE)  # 접근 위치로 이동
            time.sleep(2.)

            # 2) Yaw(회전) 고정
            cur = self.get_current_posj()  # 현재 조인트 값 읽기
            cur[-1] = target_pos[-1]       # 마지막 조인트(Yaw)만 목표값으로 변경
            self.movej(cur, 20, 20)        # 조인트 이동
            time.sleep(2.)

            # 3) 그리퍼 열기
            self.get_logger().info("✋ Step2: 그리퍼 열기")
            if self.gripper:
                try:
                    time.sleep(2.)
                    self.gripper.move(100)  # 그리퍼를 100 위치로 이동(열기)
                    time.sleep(2.)
                except Exception as e:
                    self.get_logger().warn(f"Gripper open 실패: {e}")
            time.sleep(2.)

            # 4) 하강
            self.get_logger().info("⬇️ Step3: 하강")
            self.movel(target_pos, vel, acc, ref=self.DR_BASE)  # 목표 위치로 하강
            time.sleep(2.)

            # 5) 물체 잡는 세기
            self.get_logger().info(f"🤏 Step4: Grasp {object_name}")

            if self.gripper:
                try:
                    cls = object_name.lower()  # 물체 클래스명 소문자 변환
                    # 클래스별 그리퍼 값 설정
                    if "cutlery" in cls:
                        grip_value = 745
                    elif "cup" in cls:
                        grip_value = 280
                    elif "dish" in cls:
                        grip_value = 750
                    else:
                        grip_value = 700

                    time.sleep(2.)
                    self.gripper.move(grip_value)  # 그리퍼 닫기
                    time.sleep(2.)

                except Exception as e:
                    self.get_logger().warn(f"Gripper grasp 실패: {e}")

        except Exception as e:
            self.get_logger().error(f"Pick & Place 시퀀스 실패: {e}")


    # ======================================================================
    #                            생성자
    # ======================================================================
    def __init__(self, movej, movel, get_current_posj, mwait, DR_BASE):
        """
        PickPlaceService 클래스 생성자
        - 로봇 제어 함수 및 그리퍼 초기화
        - 서비스 등록 및 홈 위치 이동
        """
        super().__init__("robot_string_to_move_to_point")  # ROS 2 노드명 지정

        # 1) 그리퍼 컨트롤러 초기화 (DRL 기반)
        self.gripper = 1
        if GripperController is not None:
            self.gripper = GripperController(self, namespace=ROBOT_ID)  # 그리퍼 객체 생성
            ok = self.gripper.initialize()  # 그리퍼 초기화

        # 2) 로봇 제어 함수 할당
        self.movej = movej  # 조인트 이동 함수
        self.movel = movel  # 직선 이동 함수
        self.get_current_posj = get_current_posj  # 현재 조인트 값 읽기 함수
        self.mwait = mwait  # 대기 함수
        self.DR_BASE = DR_BASE  # 기준 좌표계

        # 3) 서비스 등록 (StringToString 타입)
        self.srv = self.create_service(
            StringToString,
            "/move_to_box",
            self.callback
        )

        self.home = [0, 0, 90, 0, 90, 0]  # 홈(기본) 조인트 위치

        # 4) 그리퍼 초기화 재시도 및 결과 로그
        # self.gripper = None
        # if GripperController is not None:
        #     try:
        #         self.gripper = GripperController(self, namespace=ROBOT_ID)
        #         ok = self.gripper.initialize()

        #         if ok:
        #             self.get_logger().info("🔧 Gripper initialized successfully.")
        #         else:
        #             self.get_logger().warn("⚠️ Gripper initialization FAILED!")

        #     except Exception as e:
        #         self.get_logger().warn(f"⚠️ Gripper initialization exception: {e}")

        # 5) 홈 위치로 이동
        self.get_logger().info("🏠 Moving to Home Position...")
        time.sleep(2.)
        self.movej(self.home, VELOCITY, ACC)
        time.sleep(2.)  # 5초 대기 (mwait 대신)
        self.get_logger().info(f"→ Home Joint: {self.home}")
        # # 홈 위치에서 그리퍼 열기
        # if self.gripper:
        #     try:
        #         self.get_logger().info("✋ Opening Gripper at Home...")
        #         time.sleep(2.)
        #         self.gripper.move(100)
        #         time.sleep(2.)
        #     except Exception as e:
        #         self.get_logger().warn(f"Gripper open at Home 실패: {e}")
        # time.sleep(2.)


        # 6) 수거함 위치들 미리 정의
        self.cutlery_up = [455, 70, 300, 90, -180, 90]   # 커틀러리 pick up 위치
        self.cutlery_down = [455, 70, 145, 90, -180, 90] # 커틀러리 pick down 위치

        self.dish_up = [560, -10, 320, 41, 148, -20]       # 접시 pick up 위치
        self.dish_down = [560, -10, 180, 41, 148, -20]      # 접시 pick down 위치
        self.dish_down_after = [560, -35, 220, 41, 148, -20]   # 접시 pick down 후 안전 위치
        self.dish_after_up = [560, 20, 300, 90, 180, 0]        # 접시 수거함 정리 후 정렬   

        self.cup_up = [660, -45, 300, 38.7, -180, 38.52] # 컵 pick up 위치
        self.cup_down = [660, -45, 170, 38.7, -180, 38.52] # 컵 pick down 위치

        self.get_logger().info("🔥 Service Ready!")  # 서비스 준비 완료 로그


    # ======================================================================
    #                            서비스 콜백
    # ======================================================================
    def callback(self, request, response):
        """
        서비스 콜백 함수
        - 외부에서 서비스 요청이 들어오면 실행됨
        - 입력값 파싱, 좌표 변환, 로봇 이동, 그리퍼 동작 등 수행
        """
        import time
        time.sleep(10)
        self.get_logger().info("==========================================")
        self.get_logger().info("📩 SERVICE REQUEST RECEIVED")
        self.get_logger().info("==========================================")

        data = ast.literal_eval(request.input)  # 입력 문자열을 dict로 변환

        center = data["center"]      # 물체 중심 좌표
        angle = data["angle_deg"]    # 물체 각도
        cls = data["class"]          # 물체 클래스명

        self.get_logger().info(f"📌 center      = {center}")
        self.get_logger().info(f"📌 angle_deg   = {angle}")
        self.get_logger().info(f"📌 class       = {cls}, type = {type(cls)}")

        # 1) pick 동작 시퀀스: x,y,z=300mm로 이동 → z=인식된 z값으로 이동 → 그리퍼 동작 → z=300mm로 복귀 → 수거함 위치 이동
        # x, y, z = center[0], center[1], center[2]

        # # dish일 때 x좌표를 95mm 빼고, y좌표를 200mm 빼고, 각도 90, rx,ry,rz 고정
        # if "dish" in cls:
        #     x = x - 0.095
        #     y = y - 0.2
        #     rx, ry, rz = 41, 148, -20
        #     self.get_logger().info(f"dish detected: y좌표를 97.5mm 빼기 → {y}, 각도 90, rx,ry,rz 고정")
        #     pos_300 = [x * 1000, y * 1000, 320, rx, ry, rz]
        #     self.get_logger().info(f"➡️  Moving to (x, y, z=300): {pos_300}")
        #     self.movel(pos_300, VELOCITY, ACC)
        #     time.sleep(2.)

        #     z_offset_mm = z * 1000 + 140
        #     pos_z_offset = [x * 1000, y * 1000, z_offset_mm, rx, ry, rz]
        #     self.get_logger().info(f"➡️  물체 위치로 이동 (x, y, z={z_offset_mm}): {pos_z_offset} (z + 140mm offset)")
        #     self.movel(pos_z_offset, VELOCITY, ACC)
        #     time.sleep(2.)

        #     # # 2) 조인트 각도 보정 (기존대로 유지)
        #     # cur = self.get_current_posj()
        #     # cur[-1] = angle
        #     # self.get_logger().info(f"➡️  movej to adjusted joint: {cur}")
        #     # self.movej(cur, VELOCITY, ACC)
        #     # time.sleep(2.        # if "cutlery" in cls:
        #     time.sleep(2.)
            
            
        #     pos_300 = [x * 1000 -30, y * 1000 -25, 220, 90, 180, 0]
        #     self.get_logger().info(f"➡️  Moving to cutlery (x, y, z=300): {pos_300}")
        #     self.movel(pos_300, VELOCITY, ACC)
        #     time.sleep(2.)

        #     cur = self.get_current_posj()
        #     cur[-1] = 270 - angle
        #     self.get_logger().info(f"➡️  movej to adjusted joint (cutlery): {cur}")
        #     self.movej(cur, VELOCITY, ACC)
        #     time.sleep(2.)

        #     z_offset_mm = z * 1000 + 150
        #     y_offset_mm = y * 1000 - 25
        #     pos_z_offset = [x * 1000 -30, y_offset_mm -10, z_offset_mm, 90, 180, 0]
        #     self.get_logger().info(f"➡️  물체 위치로 이동 (x, y, z={z_offset_mm}): {pos_z_offset} (z + offset)")
        #     ret = self.movel(pos_z_offset, VELOCITY, ACC)
        #     self.get_logger().info(f"movel result: {ret}")
        #     time.sleep(2.)

        # elif "cup" in cls:
        #     time.sleep(2.)
        #     pos_300 = [x * 1000 -30, y * 1000 -25, 300, 90, 180, 0]
        #     self.get_logger().info(f"➡️  Moving to cup (x, y, z=300): {pos_300}")
        #     self.movel(pos_300, VELOCITY, ACC)
        #     time.sleep(2.)

        #     z_offset_mm = 190
        #     y_offset_mm = y * 1000 
        #     self.get_logger().info(f"cup detected: y에 40mm, z에 150mm 오프셋 적용")
        #     pos_z_offset = [x * 1000 -30, y_offset_mm -25, z_offset_mm, 90, 180, 0]
        #     self.get_logger().info(f"➡️  물체 위치로 이동 (x, y, z={z_offset_mm}): {pos_z_offset} (z + offset)")
        #     ret = self.movel(pos_z_offset, VELOCITY, ACC)
        #     self.get_logger().info(f"movel result: {ret}")
        #     time.sleep(2.)


        # # 3) 클래스별 그리퍼 값 결정
        # if "cutlery" in cls:
        #     grip_value = 745
        # elif "cup" in cls:
        #     grip_value = 280
        # elif "dish" in cls:
        #     grip_value = 750

        # # 4) 그리퍼 닫기 (grip)
        # if self.gripper:
        #     try:
        #         self.get_logger().info(f"🤏 물체 잡기 → 세기 = {grip_value}")
        #         time.sleep(2.)
        #         self.gripper.move(grip_value)
        #         time.sleep(2.)
        #     except Exception as e:
        #         self.get_logger().warn(f"⚠️ Failed to grasp: {e}")
        # time.sleep(2.)


        # # 5) z=300mm로 복귀
        # pos_400 = pos_300.copy()
        # pos_400[2] = 400
        # self.get_logger().info("⬆️ 물체좌표의 z=400mm")
        # time.sleep(2.)
        # self.movel(pos_400, VELOCITY, ACC)
        # time.sleep(2.)

        # 6) 수거함 위치로 이동 (클래스별)
        if "cutlery" in cls:
            bin_up = self.cutlery_up
            bin_down = self.cutlery_down
            open_value = 650
            bin_name = "식기류"
        elif "dish" in cls:
            bin_up = self.dish_up
            bin_down = self.dish_down
            open_value = 350
            bin_name = "그릇"
        elif "cup" in cls:
            bin_up = self.cup_up
            bin_down = self.cup_down
            open_value = 200
            bin_name = "컵"
        else:
            bin_up = self.cutlery_up
            bin_down = self.cutlery_down
            open_value = 100
            bin_name = "기본"

        self.get_logger().info(f"🚚 {bin_name} 수거함 위로 이동")
        self.movel(bin_up, VELOCITY, ACC, ref=self.DR_BASE)
        time.sleep(2.)

        self.get_logger().info(f"🚚 {bin_name} 수거함 속으로 ~")
        self.movel(bin_down, VELOCITY, ACC, ref=self.DR_BASE)
        time.sleep(2.)        # 그리퍼 열기 (클래스별)
        if self.gripper:
            try:
                self.get_logger().info(f"✋ {bin_name} 수거함에 넣기 (그리퍼 {open_value}으로 열기)")
                self.gripper.move(open_value)
            except Exception as e:
                self.get_logger().warn(f"Gripper open at bin 실패: {e}")
        time.sleep(2.)

        self.get_logger().info(f"🚚 {bin_name} 수거 완료 !")
        self.movel(bin_up, VELOCITY, ACC, ref=self.DR_BASE)
        time.sleep(2.)

        # 7) 홈 위치로 복귀
        self.get_logger().info("🏠 Returning to Home Position...")
        self.movej(self.home, VELOCITY, ACC)
        time.sleep(2.)
        self.gripper.move(100)  # 홈 위치에서 그리퍼 열기
        time.sleep(2.)
        self.get_logger().info(f"→ Home Joint: {self.home}")

        response.output = str(data)  # 서비스 응답값 설정
        return response



# ======================================================================
#                               MAIN
# ======================================================================
def main(args=None):
    """
    메인 함수
    - ROS 2 초기화 및 노드 생성
    - PickPlaceService 노드 실행
    """

    DR_init.__dsr__id = ROBOT_ID         # 로봇 ID 설정
    DR_init.__dsr__model = ROBOT_MODEL   # 로봇 모델 설정

    rclpy.init(args=args)                # ROS 2 초기화

    node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)  # ROS 2 노드 생성
    DR_init.__dsr__node = node           # DR_init에 노드 객체 등록

    # 로봇 제어 함수 import (movej, movel 등)
    from DSR_ROBOT2 import movej, movel, get_current_posj, mwait, DR_BASE

    # PickPlaceService 노드 생성 및 실행
    service_node = PickPlaceService(movej, movel, get_current_posj, mwait, DR_BASE)

    rclpy.spin(service_node)              # 서비스 노드 실행 (이벤트 루프)

    # 종료 시 노드 파괴 및 ROS 2 종료
    service_node.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()  # 메인 함수 실행
