#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import os

class PiperLauncher(Node):
    def __init__(self):
        super().__init__('piper_launcher')
        self.get_logger().info("🚀 Launching Piper server...")

        try:
            os.chdir('/home/daniil/piper')  # カレントディレクトリを移動
            self.process = subprocess.Popen([
                "docker", "run", "--rm", "-p", "5000:5000", "-v",
                f"{os.getcwd()}/models:/app/models",
                "piper-server"
            ])
            self.get_logger().info("✅ Piper server started.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start Piper: {e}")

def main():
    rclpy.init()
    node = PiperLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
