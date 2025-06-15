import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_2')

        # Souscription au topic /cmd_vel
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publication du feedback des roues
        self.feedback_publisher = self.create_publisher(String, 'wheel_feedback', 10)

        try:
            # Connexion série : Rear = moteurs, Front = direction
            self.serial_rear = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
            self.serial_front = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("✅ Ports série ouverts avec succès")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Erreur ouverture série : {e}")
            self.serial_rear = None
            self.serial_front = None

        # Lancer la lecture du feedback en parallèle
        if self.serial_rear:
            threading.Thread(target=self.read_feedback, daemon=True).start()

    def cmd_vel_callback(self, msg):
        try:
            if self.serial_rear:
                self.serial_rear.write(f"{msg.linear.x:.3f}\n".encode())
            if self.serial_front:
                self.serial_front.write(f"{msg.angular.z:.3f}\n".encode())

            self.get_logger().info(f"📤 Commande envoyée -> Vx: {msg.linear.x:.2f} m/s, Wz: {msg.angular.z:.2f} rad/s")
        except Exception as e:
            self.get_logger().error(f"❌ Erreur envoi commande : {e}")

    def read_feedback(self):
        while rclpy.ok():
            try:
                line = self.serial_rear.readline().decode().strip()
                parts = line.split()

                if len(parts) == 3 and parts[0] == "SPEED":
                    right, left = map(float, parts[1:])
                    msg = String()
                    msg.data = f"R: {right:.2f}, L: {left:.2f}"
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
        if node.serial_rear:
            node.serial_rear.close()
        if node.serial_front:
            node.serial_front.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
