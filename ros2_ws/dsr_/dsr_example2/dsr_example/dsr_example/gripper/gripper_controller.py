import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import textwrap


DRL_BASE_CODE = """
g_slaveid = 0
flag = 0
def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid
    
def modbus_fc06(address, value):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (6).to_bytes(1, byteorder='big')
    data += (address).to_bytes(2, byteorder='big')
    data += (value).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
    
def modbus_fc16(startaddress, cnt, valuelist):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (16).to_bytes(1, byteorder='big')
    data += (startaddress).to_bytes(2, byteorder='big')
    data += (cnt).to_bytes(2, byteorder='big')
    data += (2 * cnt).to_bytes(1, byteorder='big')
    for i in range(0, cnt):
        data += (valuelist[i]).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
    
def recv_check():
    size, val = flange_serial_read(0.1)
    if size > 0:
        return True, val
    else:
        tp_log("CRC Check Fail")
        return False, val
def gripper_move(stroke):
    flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
    wait(1.0) 

# ---- init serial & torque/current ----
while True:
    flange_serial_open(
        baudrate=57600,
        bytesize=DR_EIGHTBITS,
        parity=DR_PARITY_NONE,
        stopbits=DR_STOPBITS_ONE,
    )

    modbus_set_slaveid(1)

    # 256(40257) Torque enable
    # 275(40276) Goal Current
    # 282(40283) Goal Position

    flange_serial_write(modbus_fc06(256, 1))   # torque enable
    flag, val = recv_check()

    flange_serial_write(modbus_fc06(275, 400)) # goal current
    flag, val = recv_check()

    if flag is True:
        break

    flange_serial_close()
"""

class GripperController:
    def __init__(self, node: Node, namespace: str = "dsr01", robot_system: int = 0):
        self.node = node
        self.robot_system = robot_system
        self.cli = self.node.create_client(DrlStart, f"/{namespace}/drl/drl_start")

        self.node.get_logger().info(f"Waiting for service /{namespace}/drl/drl_start...")
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info("Service not available, waiting again...")
        self.node.get_logger().info("GripperController is ready.")

    def _send_drl_script(self, code: str) -> bool:
        req = DrlStart.Request()
        req.robot_system = self.robot_system
        req.code = code
        future = self.cli.call_async(req)
        
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is not None:
            return bool(future.result().success)
        else:
            self.node.get_logger().error(f"Service call failed: {future.exception()}")
            return False

    def initialize(self) -> bool:
        self.node.get_logger().info("Initializing gripper connection...")
        task_code = textwrap.dedent("""
            flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
            modbus_set_slaveid(1)
            flange_serial_write(modbus_fc06(256, 1))
            recv_check()
            flange_serial_write(modbus_fc06(275, 400))
            recv_check()
        """)
        init_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(init_script)
        if success:
            self.node.get_logger().info("Gripper connection initialized successfully.")
        else:
            self.node.get_logger().error("Failed to initialize gripper connection.")
        return success

    def move(self, stroke: int) -> bool:
        self.node.get_logger().info(f"Moving gripper to stroke: {stroke}")
        task_code = textwrap.dedent(f"""
            gripper_move({stroke})
        """)
        move_script = f"{DRL_BASE_CODE}\n{task_code}"
        success = self._send_drl_script(move_script)
        if success:
            self.node.get_logger().info(f"Gripper move command for {stroke} sent successfully.")
        else:
            self.node.get_logger().error("Failed to send gripper move command.")
        return success

    def open(self) -> bool:
        self.node.get_logger().info(f"Moving gripper to open")
        return self.move(0)

    def close(self) -> bool:
        self.node.get_logger().info(f"Moving gripper to close")
        return self.move(700)

    def terminate(self) -> bool:
        self.node.get_logger().info("Terminating gripper connection...")
        terminate_script = "flange_serial_close()"
        success = self._send_drl_script(terminate_script)
        if success:
            self.node.get_logger().info("Gripper connection terminated successfully.")
        else:
            self.node.get_logger().error("Failed to terminate gripper connection.")
        return success
      