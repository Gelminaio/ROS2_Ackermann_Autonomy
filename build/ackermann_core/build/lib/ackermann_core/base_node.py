import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class AckermannBaseNode(Node):
    WHEELBASE_M = 0.20
    MAX_STEER_OFFSET = 350
    MAX_STEER_RAD = 0.60

    def __init__(self):
        super().__init__("ackermann_base_node")

        try:
            from hardware import ackermann_hal
        except ImportError:
            from ackermann_core.hardware import ackermann_hal

        self._hal = ackermann_hal

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._current_steering_offset = 0
        self._last_update_time = self.get_clock().now()

        self._cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._cmd_vel_callback,
            10,
        )

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_timer = self.create_timer(0.02, self._odom_timer_callback)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        speed = int(msg.linear.x * 100)
        steering_offset = int(msg.angular.z * -350)
        steering_offset = max(-self.MAX_STEER_OFFSET, min(self.MAX_STEER_OFFSET, steering_offset))

        self._current_steering_offset = steering_offset
        self._hal.desired_speed = speed
        self._hal.request_steering(steering_offset)

    def _odom_timer_callback(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_update_time).nanoseconds * 1e-9
        self._last_update_time = now

        if dt <= 0.0:
            return

        dl, dr, v_lin = self._hal.get_odometry()

        steer_angle = (self._current_steering_offset / float(self.MAX_STEER_OFFSET)) * self.MAX_STEER_RAD
        omega = 0.0
        if abs(steer_angle) > 1e-6:
            omega = v_lin * math.tan(steer_angle) / self.WHEELBASE_M

        self._theta += omega * dt
        self._x += v_lin * math.cos(self._theta) * dt
        self._y += v_lin * math.sin(self._theta) * dt

        qz = math.sin(self._theta * 0.5)
        qw = math.cos(self._theta * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(v_lin)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega

        self._odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
