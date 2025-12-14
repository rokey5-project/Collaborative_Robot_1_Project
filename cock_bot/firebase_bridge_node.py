#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Int32

# ---------------- Firebase ----------------
import firebase_admin
from firebase_admin import credentials, db

SERVICE_ACCOUNT_KEY_PATH = "/home/rokey/cobot_json/cobot1-8e870-firebase-adminsdk-fbsvc-3414a26a62.json"
DATABASE_URL = "https://cobot1-8e870-default-rtdb.asia-southeast1.firebasedatabase.app"


class FirebaseBridgeNode(Node):

    def __init__(self):
        super().__init__("firebase_bridge_node")

        # ---- internal states
        self.is_shutdown = False
        self.last_menu = None

        # ---------------- ROS2 Publishers (UI -> FSM) ----------------
        self.start_pub = self.create_publisher(Bool, "/order/start", 10)
        self.menu_pub  = self.create_publisher(String, "/order/menu", 10)

        # ---------------- ROS2 Subscribers (Robot/FSM -> UI) ----------------
        self.create_subscription(String, "/robot/state", self.state_cb, 10)
        self.create_subscription(Int32, "/robot/progress", self.progress_cb, 10)
        self.create_subscription(String, "/robot/error", self.error_cb, 10)

        # ---------------- Firebase Init ----------------
        try:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {
                "databaseURL": DATABASE_URL
            })
            self.get_logger().info("Firebase initialized")
        except ValueError:
            self.get_logger().info("Firebase already initialized")

        self.db_root = db.reference("/")

        # ---------------- Firebase Listener Thread ----------------
        self.firebase_thread = threading.Thread(
            target=self.firebase_listener,
            daemon=True
        )
        self.firebase_thread.start()

        self.get_logger().info("firebase_bridge_node started")

        # ---------------- Firebase to ros2 ----------------
        
    def firebase_listener(self):
        """
        Poll Firebase commands and convert them to ROS2 topics
        """
        while not self.is_shutdown:
            try:
                # UI -> Firebase
                start_cmd = self.db_root.child("order/start").get()
                menu_cmd  = self.db_root.child("order/menu").get()

                # ---- Start command (edge-trigger)
                if start_cmd is True:
                    self.start_pub.publish(Bool(data=True))
                    self.db_root.child("order/start").set(False)
                    self.get_logger().info("[UI] Start command received")

                # ---- Menu selection (change-trigger)
                if menu_cmd and menu_cmd != self.last_menu:
                    self.menu_pub.publish(String(data=menu_cmd))
                    self.last_menu = menu_cmd
                    self.get_logger().info(f"[UI] Menu selected: {menu_cmd}")

                time.sleep(0.5)

            except Exception as e:
                self.get_logger().error(f"Firebase listener error: {e}")
                time.sleep(1.0)

         # ---------------- ros2 to firebase ----------------
    def state_cb(self, msg: String):
        self.db_root.child("status/state").set(msg.data)

    def progress_cb(self, msg: Int32):
        self.db_root.child("status/progress").set(msg.data)

    def error_cb(self, msg: String):
        self.db_root.child("status/error").set(msg.data)

         # ---------------- shutdown handling ----------------
    def destroy_node(self):
        self.is_shutdown = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
