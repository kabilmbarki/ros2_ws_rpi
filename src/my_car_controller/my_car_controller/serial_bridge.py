import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_mega')

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.feedback_publisher = self.create_publisher(String, 'wheel_feedback', 10)

        try:
            self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("✅ Port série ouvert (/dev/ttyACM0)")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Erreur ouverture port série : {e}")
            self.serial = None

        if self.serial:
            threading.Thread(target=self.read_feedback, daemon=True).start()

    def cmd_vel_callback(self, msg):
        try:
            if self.serial:
                cmd_str = f"{msg.linear.x:.3f} {msg.angular.z:.3f}\n"
                self.serial.write(cmd_str.encode())
                self.get_logger().info(f"📤 Cmd -> Vx: {msg.linear.x:.2f}, Wz: {msg.angular.z:.2f}")
        except Exception as e:
            self.get_logger().error(f"❌ Erreur envoi commande : {e}")

    def read_feedback(self):
        while rclpy.ok():
            try:
                line = self.serial.readline().decode().strip()
                parts = line.split()

                if len(parts) == 4 and parts[0] == "SPEED":
                    right = float(parts[1])
                    left = float(parts[2])
                    yaw_rate = float(parts[3])

                    msg = String()
                    msg.data = f"SPEED {right:.2f} {left:.2f} {yaw_rate:.2f}"
                    self.feedback_publisher.publish(msg)
                    self.get_logger().info(f"📥 Feedback reçu -> {msg.data}")
            except Exception as e:
                self.get_logger().error(f"❌ Erreur lecture feedback : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

