import rclpy
import numpy as np
from rclpy.time import Time
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler


class Vel_Odometry(LifecycleNode):

    def __init__(self):
        super().__init__("odometry_node")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"Transitioning from '{state.label}' to 'inactive' state."
        )
        self.prev_odom = [None, None, None, None]

        self._odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self._odometry_publisher = self.create_publisher(Odometry, "/odometry", 10)

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        return super().on_activate(state)

    def _publish_odometry(self, z_v: float, z_w: float) -> None:
        """Publishes odometry measurements in a nav_msgs.msg.Odometry message.

        Args:
            z_v: Linear velocity of the robot center [m/s].
            z_w: Angular velocity of the robot center [rad/s].

        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.twist.twist.linear.x = z_v
        odom_msg.twist.twist.angular.z = z_w

        self._odometry_publisher.publish(odom_msg)

    def odom_callback(self, msg: Odometry):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat_w = msg.pose.pose.orientation.w
        quat_x = msg.pose.pose.orientation.x
        quat_y = msg.pose.pose.orientation.y
        quat_z = msg.pose.pose.orientation.z

        _, _, th_h = quat2euler((quat_w, quat_x, quat_y, quat_z))

        t1 = Time.from_msg(msg.header.stamp).nanoseconds

        if self.prev_odom[0] == None:
            self.prev_odom[0] = x
            self.prev_odom[1] = y
            self.prev_odom[2] = th_h
            self.prev_odom[3] = t1

        else:
            z_v = np.sqrt((x - self.prev_odom[0]) ** 2 + (y - self.prev_odom[1]) ** 2) / ((t1 - self.prev_odom[2]) * 1e-9)
            z_w = (th_h - self.prev_odom[1]) / ((t1 - self.prev_odom[2]) * 1e-9)
            # self._publish_odometry(z_v, z_w)
            msg.twist.twist.linear.x = z_v
            msg.twist.twist.angular.z = z_w

            self._odometry_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    odometry_node = Vel_Odometry()

    rclpy.spin(odometry_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odometry_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
