import asyncio
from queue import Queue
import threading
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn

# --------- 글로벌 상태 ---------
app = FastAPI()

# ROS2에서 온 메시지를 담아둘 thread-safe 큐
message_queue: "Queue[str]" = Queue()

# 현재 연결된 WebSocket 클라이언트들 (/ws/keywords용)
connected_clients: set[WebSocket] = set()

# BridgeNode를 다른 쓰레드(WebSocket)에서 접근하기 위한 전역 포인터
bridge_node: "BridgeNode | None" = None

# --------- ROS2 Bridge 노드 ---------
class BridgeNode(Node):
    def __init__(self):
        super().__init__("ros_web_bridge")
        self.robot_state = ""
        # keyword_topic → React
        # llm으로부터 받는 친구
        self.keyword_sub = self.create_subscription(
            String,
            "keyword_topic",
            self.keyword_callback,
            10,
        )

        # voice_state → React
        # 마이크상태 받는 친구
        self.voice_state_sub = self.create_subscription(
            String,
            "voice_state",
            self.voice_state_callback,
            10,
        )

        # 🔥 React → ROS : 로봇 도착 이벤트 토픽
        #   - 토픽명: "robot_event"
        #   - 메시지 예: "move, arrived"
        # gui 이동상태 쏘는 친구
        self.robot_event_pub = self.create_publisher(
            String,
            "robot_status",
            10,
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

        # 0번 받는 친구
        self.robot_return_sub = self.create_subscription(
            String,
            "/robot_command",
            self.robot_return_state_callback,
            10,
        )

        self.get_logger().info("ROS Web Bridge Node started.")

    def keyword_callback(self, msg: String):
        # msg.data는 이미 JSON string (get_keyword 쪽에서 json.dumps 한 것)
        data = msg.data
        self.get_logger().info(f"[BRIDGE] keyword_topic: {data}")
        message_queue.put(data)

    def voice_state_callback(self, msg: String):
        # msg.data: "waiting" or "listening"
        payload = json.dumps(
            {
                "type": "voice_state",
                "state": msg.data,
            },
            ensure_ascii=False,
        )
        self.get_logger().info(f"[BRIDGE] voice_state: {payload}")
        message_queue.put(payload)
    # def 대충 콜백(self, msg: String):
    #     self.robot_event = msg.data
    #     if self.robot_event != 종료:
    #         self.robot_state = moving

    def robot_return_state_callback(self, msg: String):
        # msg.data는 이미 JSON string (get_keyword 쪽에서 json.dumps 한 것)
        data = msg.data
        self.get_logger().info(f"[BRIDGE] robot_return: {data}")
        message_queue.put(data)

    def timer_callback(self):
        msg = String()
        msg.data = self.robot_state  # "", "moving", "arrived"
        self.robot_event_pub.publish(msg)
        

def ros_spin():
    global bridge_node
    rclpy.init()
    bridge_node = BridgeNode()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    bridge_node = None
    rclpy.shutdown()


# --------- WebSocket 브로드캐스트 (/ws/keywords용) ---------
async def broadcast_to_clients(data: str):
    """현재 연결된 모든 /ws/keywords WebSocket 클라이언트에게 data 전송"""
    dead_clients = []
    for ws in list(connected_clients):
        try:
            await ws.send_text(data)
        except Exception:
            dead_clients.append(ws)
    for ws in dead_clients:
        connected_clients.discard(ws)


async def message_relay_task():
    """message_queue에서 데이터를 빼서 /ws/keywords WebSocket으로 브로드캐스트하는 백그라운드 태스크"""
    loop = asyncio.get_event_loop()
    while True:
        # blocking queue.get()를 executor에서 돌려서 non-blocking처럼 사용
        data = await loop.run_in_executor(None, message_queue.get)
        await broadcast_to_clients(data)


@app.on_event("startup")
async def startup_event():
    # FastAPI 시작될 때 백그라운드 태스크 시작
    asyncio.create_task(message_relay_task())


# --------- WebSocket 엔드포인트: /ws/keywords ---------
@app.websocket("/ws/keywords")
async def websocket_keywords(websocket: WebSocket):
    await websocket.accept()
    connected_clients.add(websocket)
    print("WebSocket /ws/keywords client connected")

    try:
        while True:
            # 클라이언트에서 오는 메시지가 필요 없으면 그냥 대기만
            await websocket.receive_text()
    except WebSocketDisconnect:
        print("WebSocket /ws/keywords client disconnected")
        connected_clients.discard(websocket)


# --------- WebSocket 엔드포인트: /ws/robot_events ---------
# React(App.jsx) → 도착 이벤트(arrived) 받는 용도
@app.websocket("/ws/robot_events")
async def websocket_robot_events(websocket: WebSocket):
    await websocket.accept()
    print("WebSocket /ws/robot_events client connected")

    try:
        while True:
            msg = await websocket.receive_text()
            print("[/ws/robot_events] raw:", msg)

            try:
                data = json.loads(msg)
            except json.JSONDecodeError:
                print("⚠ invalid JSON from client")
                continue

            # 기대 포맷:
            # {
            #   "type": "robot_event",
            #   "event": "arrived",
            #   "table": 3
            # }
            if data.get("type") == "robot_event" and data.get("event") == "arrived":
                table = data.get("table")
                print(f"🚀 React says robot arrived at table: {table}")

                # ROS 토픽으로 전달
                global bridge_node
                if bridge_node is not None:
                    if table == 0:
                        bridge_node.robot_state = "waiting"
                    else:
                        bridge_node.robot_state = "arrived"
                    bridge_node.get_logger().info(
                        f"[BRIDGE] robot_state changed to: {bridge_node.robot_state}"
                    )
                else:
                        print("⚠ bridge_node is None, cannot publish robot_event")

    except WebSocketDisconnect:
        print("WebSocket /ws/robot_events client disconnected")


def main(args=None):
    """ROS2용 엔트리 포인트 (ros2 run 에서 사용)"""
    # ROS2는 별도 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # FastAPI(Uvicorn) 실행
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    # python ros_web_bridge.py 로 직접 실행할 때
    main()