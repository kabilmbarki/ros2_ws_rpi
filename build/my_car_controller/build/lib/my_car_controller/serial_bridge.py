import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for feedback from both Arduinos
        self.feedback_publisher = self.create_publisher(String, 'wheel_feedback', 10)

        # Try to connect to both serial ports
        self.serial_ports = []

        for port_name in ['/dev/ttyACM0', '/dev/ttyACM1']:
            try:
                port = serial.Serial(port_name, 115200, timeout=1)
                self.serial_ports.append(port)
                self.get_logger().info(f"Serial port {port_name} opened successfully.")
                # Start a thread for each port
                threading.Thread(target=self.read_feedback, args=(port, port_name), daemon=True).start()
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port {port_name}: {e}")

    def cmd_vel_callback(self, msg):
        """Send velocity commands to both Arduinos."""
        command = f"{msg.linear.x:.3f} {msg.angular.z:.3f}\n"
        for port in self.serial_ports:
            try:
                port.write(command.encode())
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial port: {e}")
        self.get_logger().info(f"Sent to all: {command.strip()}")

    def read_feedback(self, port, port_name):
        """Read and publish feedback from a specific Arduino."""
        while rclpy.ok():
            try:
                line = port.readline().decode().strip()
                parts = line.split()
                if len(parts) == 2:
                    right_speed, left_speed = map(float, parts)
                    feedback_msg = String()
                    feedback_msg.data = f"{port_name} -> R: {right_speed:.2f}, L: {left_speed:.2f}"
                    self.feedback_publisher.publish(feedback_msg)
                    self.get_logger().info(f"Published: {feedback_msg.data}")
            except Exception as e:
                self.get_logger().error(f"Serial read error on {port_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        for port in node.serial_ports:
            port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '_main_':
    main()
