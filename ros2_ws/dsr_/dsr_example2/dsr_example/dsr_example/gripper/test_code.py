import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart
import DR_init

from dsr_example.gripper.gripper_controller import GripperController


def main(args=None):
    rclpy.init(args=args)
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "e0509"
    VEL = 30
    ACC = 30
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node("gripper_example_py", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, posj, movej, wait

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    gripper = None
    try:
        gripper = GripperController(node=node, namespace=ROBOT_ID)
        if not gripper.initialize():
            node.get_logger().error("Gripper initialization failed. Exiting.")
            return

        node.get_logger().info("Sending a priming command to wake up the gripper...")
        wait(2)

        # p_start = posj(0, 0, 90, 0, 90, 0)
        # movej(p_start, VEL, ACC)
        # gripper.move(700)
        # wait(2)

        # p1_joint = posj(45, 0, 90, 0, 90, 0)
        # p2_joint = posj(-45, 0, 90, 0, 90, 0)

        print("이동합니다.")
        # # movej(p1_joint, VEL, ACC)
        # gripper.move(0)
        # wait(2)

        print("이동합니다.")
        # movej(p2_joint, VEL, ACC)
        # gripper.move(700)
        # wait(2)

        print("복귀합니다.")
        # movej(p_start, VEL, ACC)
        gripper.move(100)
        wait(2)

    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        if gripper:
            gripper.terminate()
        rclpy.shutdown()
        node.get_logger().info("Shutdown complete.")


if __name__ == "__main__":
    main()