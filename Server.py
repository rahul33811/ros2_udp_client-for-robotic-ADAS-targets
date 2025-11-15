import rclpy
from rclpy.node import Node
import socket
import struct
import threading
from std_msgs.msg import Float64  # Placeholder for your actual message type

# -------------------------
# Configurable Parameters
# -------------------------
DEFAULT_UDP_IP = "0.0.0.0"          # Bind to all interfaces
DEFAULT_UDP_RECEIVE_PORT = 0        # Use 0 or replace with non-sensitive port
DEFAULT_UDP_SEND_PORT = 0           # Same as above
SCALING_FACTOR_V = 100              # Generic scaling factor

class UDPReceiverClientNode(Node):

    def __init__(self):
        super().__init__('udp_receiver_client')

        # UDP server (receiver)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((DEFAULT_UDP_IP, DEFAULT_UDP_RECEIVE_PORT))
        self.get_logger().info('UDP receiver initialized.')

        # UDP client (sender)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Replace Float64 with your actual message type
        self.subscription = self.create_subscription(
            Float64,
            'input_topic',
            self.input_callback,
            10
        )

        # Thread for receiving UDP packets
        self.receive_thread = threading.Thread(target=self.receive_udp_messages, daemon=True)
        self.receive_thread.start()

    # -------------------------
    # UDP Reception
    # -------------------------
    def receive_udp_messages(self):
        while rclpy.ok():
            try:
                data, _ = self.socket.recvfrom(1024)
                self.process_udp_message(data)
            except Exception:
                pass

    def process_udp_message(self, data):
        """ Generic message unpacking (safe for public use). """
        if len(data) < 10:
            return
        
        try:
            sync_byte, message_id, payload_len, obj_index, timestamp = struct.unpack('<BBHHL', data[:10])
            payload = data[10:]

            self.get_logger().info(
                f"Received UDP packet | ID={message_id}, Payload={len(payload)} bytes"
            )

        except Exception:
            self.get_logger().warn("Malformed UDP packet received.")

    # -------------------------
    # ROS2 Input Processing
    # -------------------------
    def input_callback(self, msg):
        """ Generic ROS2 message handler. """

        # Example payload conversion (safe placeholder)
        value = float(msg.data)

        payload = struct.pack('<d', value)

        self.client_socket.sendto(payload, (DEFAULT_UDP_IP, DEFAULT_UDP_SEND_PORT))
        self.get_logger().info("Sent UDP payload.")

    # -------------------------
    # Cleanup
    # -------------------------
    def destroy_node(self):
        self.socket.close()
        self.client_socket.close()
        super().destroy_node()

# -------------------------
# Main Entry
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = UDPReceiverClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
