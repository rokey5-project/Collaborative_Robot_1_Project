#!/usr/bin/env python3
import subprocess
import os
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Int32


class OrderManageNode(Node):

    def __init__(self):
        super().__init__("order_manage_node")

        # ---------------- Paths ----------------
        self.pkg_path = os.path.dirname(os.path.abspath(__file__))

        self.sequence = [
            ("M1_GET_SHAKER", "m1_motion_node.drl"),
            ("M2_PUT_SHAKER", "m2_put_shaker.drl"),
            ("M3_CLOSE_CAP", "m3_close_cap.drl"),
            ("M4_GET_SHAKER_AGAIN", "m4_get_shaker.drl"),
            ("M5_SHAKING", "m5_shaking.drl"),
            ("M6_PUT_SHAKER", "m6_put_shaker.drl"),
            ("M7_OPEN_CAP", "m7_twist_open_cap.drl"),
            ("M8_POUR", "m8_pour_drink.drl"),
        ]

        self.progress_map = {
            "IDLE": 0,
            "M1_GET_SHAKER": 10,
            "M2_PUT_SHAKER": 20,
            "M3_CLOSE_CAP": 30,
            "M4_GET_SHAKER_AGAIN": 40,
            "M5_SHAKING": 60,
            "M6_PUT_SHAKER": 75,
            "M7_OPEN_CAP": 85,
            "M8_POUR": 95,
            "DONE": 100,
            "ERROR": 0,
        }

        # ---------------- State ----------------
        self.is_running = False
        self.current_state = "IDLE"

        # ---------------- ROS2 ----------------
        self.create_subscription(Bool, "/order/start", self.start_cb, 10)

        self.state_pub = self.create_publisher(String, "/robot/state", 10)
        self.progress_pub = self.create_publisher(Int32, "/robot/progress", 10)
        self.error_pub = self.create_publisher(String, "/robot/error", 10)

        self.publish_state("IDLE")

        self.get_logger().info("order_manage_node started")

    # ---------------- Callbacks ----------------

    def start_cb(self, msg: Bool):
        if not msg.data:
            return

        if self.is_running:
            self.get_logger().warn("FSM already running. Ignored.")
            return

        if self.current_state == "ERROR":
            self.get_logger().info("Reset from ERROR → IDLE")
            self.transition_to("IDLE")

        self.get_logger().info("Order start received")
        threading.Thread(target=self.run_fsm, daemon=True).start()

    # ---------------- FSM CORE ----------------

    def run_fsm(self):
        self.is_running = True

        try:
            for state, script in self.sequence:
                self.transition_to(state)

                script_path = os.path.join(self.pkg_path, script)

                self.get_logger().info(f"Executing {script}")

                # DRL 실행 방식 
                result = subprocess.run(
                    ["drctl", "run", script_path],
                    capture_output=True,
                    text=True
                )

                if result.returncode != 0:
                    raise RuntimeError(result.stderr)

            self.transition_to("DONE")
            self.get_logger().info("Cocktail completed")

        except Exception as e:
            self.transition_to("ERROR")
            self.error_pub.publish(String(data=str(e)))
            self.get_logger().error(f"FSM Error: {e}")

        finally:
            self.is_running = False

            # DONE or ERROR 
            if self.current_state != "ERROR":
                self.transition_to("IDLE")

    # ---------------- Helpers ----------------

    def transition_to(self, state):
        self.current_state = state
        self.publish_state(state)

        progress = self.progress_map.get(state, 0)
        self.progress_pub.publish(Int32(data=progress))

    def publish_state(self, state):
        self.state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = OrderManageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
