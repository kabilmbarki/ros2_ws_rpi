import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math


class SerbotController(Node):
    def __init__(self):
        super().__init__('serbot_controller_2')

        # Initial pose and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ROS2 communication
        self.subscription = self.create_subscription(
            String,
            'wheel_feedback',
            self.speed_callback,
            10
        )
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Clock for dt calculation
        self.last_time = self.get_clock().now()

    def speed_callback(self, msg):
        try:
            data = msg.data.split(' ')
            if data[0] != "SPEED":
                raise ValueError("Invalid message format")

            vR = float(data[1])  # Right wheel speed (m/s)
            vL = float(data[2])  # Left wheel speed (m/s)
            yaw_deg_s = float(data[3])  # Angular velocity from MPU6050 (deg/s)

            v = (vR + vL) / 2.0
            w = math.radians(yaw_deg_s)  # Convert deg/s to rad/s

        except Exception as e:
            self.get_logger().error(f"Failed to parse speed message: {e}")
            return

        # Time step
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Update position
        dx = v * math.cos(self.theta) * dt
        dy = v * math.sin(self.theta) * dt
        dtheta = w * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = self.euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_publisher.publish(odom)

        # Broadcast TF transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(transform)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
             math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    controller = SerbotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
