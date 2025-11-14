"""
Generic ROS2 UDP Client Node
----------------------------

This open-source-friendly example demonstrates how to:
- Subscribe to a ROS2 topic carrying simple command data
- Pack the command into a structured UDP packet
- Send the packet to a specified IP/port

NOTE:
This version contains *no proprietary message types*, *no internal protocols*,
and *no company-specific identifiers*. You may customize it freely.
"""

import rclpy
from rclpy.node import Node
import socket
import threading
import struct

from std_msgs.msg import Float32MultiArray  # Generic, open-source-safe message type

# Generic IP and port (safe + replaceable)
UDP_IP = '127.0.0.1'
UDP_PORT = 5005


class UDPClientNode(Node):

    def __init__(self):
        super().__init__('udp_client')

        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subscribe to a generic open-source command topic
        self.subscription = self.create_subscription(
            Float32MultiArray,           # Generic message type
            'cmd/input',                 # Generic topic name
            self.command_callback,
            10
        )

        self.cmd_received = threading.Event()

    def command_callback(self, msg):
        """
        Expected message format: Float32MultiArray with at least 2 values:
            msg.data[0] -> acceleration command
            msg.data[1] -> curvature command
        """

        if len(msg.data) < 2:
            self.get_logger().warn("Received command with insufficient data.")
            return

        acc = msg.data[0]
        kappa = msg.data[1]

        self.get_logger().info(f"Received Command: acc={acc:.2f}, kappa={kappa:.2f}")

        # Example payload: two signed shorts + a mode flag
        payload = struct.pack('<hhB', int(acc * 100), int(kappa * 100), 1)

        # Generic example of a structured UDP header + payload
        sync_byte = 0xAA
        message_id = 1            # Generic example ID
        payload_len = len(payload)
        reserved_field = 0x0000   # Generic placeholder
        timestamp = 0             # Generic placeholder
        crc = 0                   # Generic placeholder

        header = struct.pack('<BBHHI', sync_byte, message_id, payload_len, reserved_field, timestamp)
        packet = header + payload + struct.pack('<H', crc)

        # Send packet
        self.socket.sendto(packet, (UDP_IP, UDP_PORT))

    def destroy_node(self):
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UDPClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
