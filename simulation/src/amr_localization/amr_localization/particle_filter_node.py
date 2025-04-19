import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

import message_filters
from amr_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import os
import json
import numpy as np
import time
import traceback
from transforms3d.euler import euler2quat, quat2euler

from amr_localization.particle_filter import ParticleFilter

from amr_localization.ekf import EKF


class ParticleFilterNode(LifecycleNode):
    def __init__(self):
        """Particle filter node initializer."""
        super().__init__("particle_filter")

        # Parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("enable_plot", False)
        self.declare_parameter("global_localization", True)
        self.declare_parameter("initial_pose", (0.0, 0.0, math.radians(0)))
        self.declare_parameter("initial_pose_sigma", (0.05, 0.05, math.radians(5)))
        self.declare_parameter("particles", 1000)
        self.declare_parameter("sigma_v", 0.1)
        self.declare_parameter("sigma_w", 0.1)
        self.declare_parameter("sigma_z", 0.1)
        self.declare_parameter("steps_btw_sense_updates", 10)
        self.declare_parameter("world", "lab03")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles a configuring transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'inactive' state.")

        try:
            # Parameters
            dt = self.get_parameter("dt").get_parameter_value().double_value
            self._enable_plot = self.get_parameter("enable_plot").get_parameter_value().bool_value
            global_localization = (
                self.get_parameter("global_localization").get_parameter_value().bool_value
            )
            initial_pose = tuple(
                self.get_parameter("initial_pose").get_parameter_value().double_array_value.tolist()
            )
            initial_pose_sigma = tuple(
                self.get_parameter("initial_pose_sigma")
                .get_parameter_value()
                .double_array_value.tolist()
            )
            particles = self.get_parameter("particles").get_parameter_value().integer_value
            sigma_v = self.get_parameter("sigma_v").get_parameter_value().double_value
            sigma_w = self.get_parameter("sigma_w").get_parameter_value().double_value
            sigma_z = self.get_parameter("sigma_z").get_parameter_value().double_value
            self._steps_btw_sense_updates = (
                self.get_parameter("steps_btw_sense_updates").get_parameter_value().integer_value
            )
            world = self.get_parameter("world").get_parameter_value().string_value

            # Attribute and object initializations
            self._localized = False
            self._steps = 0
            map_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", "maps", world + ".json")
            )
            self._particle_filter = ParticleFilter(
                dt,
                map_path,
                particle_count=particles,
                sigma_v=sigma_v,
                sigma_w=sigma_w,
                sigma_z=sigma_z,
                global_localization=global_localization,
                initial_pose=initial_pose,
                initial_pose_sigma=initial_pose_sigma,
            )

            if self._enable_plot:
                self._particle_filter.show("Initialization", save_figure=True)

            # Publishers
            # TODO: 3.1. Create the /pose publisher (PoseStamped message).
            self._pose_publisher = self.create_publisher(PoseStamped, "/pose", 10)

            # Subscribers
            scan_qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            self._subscribers: list[message_filters.Subscriber] = []
            self._subscribers.append(message_filters.Subscriber(self, Odometry, "/odometry"))
            self._subscribers.append(
                message_filters.Subscriber(self, LaserScan, "/scan", qos_profile=scan_qos_profile)
            )

            ts = message_filters.ApproximateTimeSynchronizer(
                self._subscribers, queue_size=10, slop=9
            )
            ts.registerCallback(self._compute_pose_callback)

            self._load_wall_params()

            # self.sigma_v = sigma_v
            # self.sigma_w = sigma_w
            # self.sigma_z = sigma_z

            self.ekf = (
                None
                if global_localization
                else EKF(
                    0.05,
                    initial_pose,
                    initial_pose_sigma,
                    sigma_v,
                    sigma_w,
                    sigma_z,
                    self.wall_params,
                )
            )

        except Exception:
            self.get_logger().error(f"{traceback.format_exc()}")
            return TransitionCallbackReturn.ERROR

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handles an activating transition.

        Args:
            state: Current lifecycle state.

        """
        self.get_logger().info(f"Transitioning from '{state.label}' to 'active' state.")

        return super().on_activate(state)

    def _compute_pose_callback(self, odom_msg: Odometry, scan_msg: LaserScan):
        """Subscriber callback. Executes a particle filter and publishes (x, y, theta) estimates.

        Args:
            odom_msg: Message containing odometry measurements.
            scan_msg: Message containing LiDAR sensor readings.

        """
        # Parse measurements
        z_v: float = odom_msg.twist.twist.linear.x
        z_w: float = odom_msg.twist.twist.angular.z
        z_scan: list[float] = scan_msg.ranges

        if not self._localized:
            # Execute particle filter
            self._execute_motion_step(z_v, z_w)
            x_h, y_h, theta_h = self._execute_measurement_step(z_scan)
            self._steps += 1
        else:
            # x = odom_msg.pose.pose.position.x
            # y = odom_msg.pose.pose.position.y
            # quat_w = odom_msg.pose.pose.orientation.w
            # quat_x = odom_msg.pose.pose.orientation.x
            # quat_y = odom_msg.pose.pose.orientation.y
            # quat_z = odom_msg.pose.pose.orientation.z
            # _, _, theta = quat2euler((quat_w, quat_x, quat_y, quat_z))
            # theta %= 2 * math.pi
            self.ekf.predict(z_v, z_w)
            x_h, y_h, theta_h = self.ekf.update(z_scan)
            self.get_logger().warn(f"EKF: {x_h}, {y_h}, {theta_h}")

        # Publish
        self._publish_pose_estimate(x_h, y_h, theta_h)

    def _execute_measurement_step(self, z_us: list[float]) -> tuple[float, float, float]:
        """Executes and monitors the measurement step (sense) of the particle filter.

        Args:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].

        Returns:
            Pose estimate (x_h, y_h, theta_h) [m, m, rad]; inf if cannot be computed.
        """
        pose = (float("inf"), float("inf"), float("inf"))

        if self._localized or not self._steps % self._steps_btw_sense_updates:
            start_time = time.perf_counter()
            self._particle_filter.resample(z_us)
            sense_time = time.perf_counter() - start_time

            self.get_logger().info(f"Sense step time: {sense_time:6.3f} s")

            if self._enable_plot:
                self._particle_filter.show("Sense", save_figure=True)

            start_time = time.perf_counter()
            self._localized, pose, covariance = self._particle_filter.compute_pose()
            clustering_time = time.perf_counter() - start_time

            if self._localized:
                # self.ekf = EKF(0.05, pose, covariance)
                self.ekf = EKF(
                    0.05,
                    pose,
                    covariance,
                    0.1,
                    0.1,
                    0.1,
                    self.wall_params,
                )

            self.get_logger().info(f"Clustering time: {clustering_time:6.3f} s")

        return pose

    def _execute_motion_step(self, z_v: float, z_w: float):
        """Executes and monitors the motion step (move) of the particle filter.

        Args:
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].
        """
        start_time = time.perf_counter()
        self._particle_filter.move(z_v, z_w)
        move_time = time.perf_counter() - start_time

        self.get_logger().info(f"Move step time: {move_time:7.3f} s")

        if self._enable_plot:
            self._particle_filter.show("Move", save_figure=True)

    def _publish_pose_estimate(self, x_h: float, y_h: float, theta_h: float) -> None:
        """Publishes the robot's pose estimate in a custom amr_msgs.msg.PoseStamped message.

        Args:
            x_h: x coordinate estimate [m].
            y_h: y coordinate estimate [m].
            theta_h: Heading estimate [rad].

        """
        # TODO: 3.2. Complete the function body with your code (i.e., replace the pass statement).
        msg = PoseStamped()

        msg.localized = self._localized
        msg.header.stamp = self.get_clock().now().to_msg()

        if self._localized:
            w, x, y, z = euler2quat(0, 0, theta_h)

            msg.pose.position.x = x_h
            msg.pose.position.y = y_h

            msg.pose.orientation.x = x
            msg.pose.orientation.y = y
            msg.pose.orientation.z = z
            msg.pose.orientation.w = w

        self._pose_publisher.publish(msg)

    def _load_wall_params(self, map_path="lab03.json"):
        """
        Loads the wall parameters from the map file.

        Args
        ----
         - map_path: Path to the map file.

        """
        pkg_dir = os.path.dirname(__file__)
        map_path = os.path.join(pkg_dir, "..", "maps", map_path)

        data = json.load(open(map_path))
        self.wall_params = []
        boundary: list[list[float]] = data["metric"]["boundary"]

        for (x1, y1), (x2, y2) in zip(boundary[:-1], boundary[1:]):
            dx = x2 - x1
            dy = y2 - y1
            length = math.sqrt(dx**2 + dy**2)

            if length < 0.1:
                continue

            alpha = math.atan2(dy, dx) + math.pi / 2
            rho = x1 * math.cos(alpha) + y1 * math.sin(alpha)
            self.wall_params.append([alpha, rho])


def main(args=None):
    rclpy.init(args=args)
    particle_filter_node = ParticleFilterNode()

    try:
        rclpy.spin(particle_filter_node)
    except KeyboardInterrupt:
        pass

    particle_filter_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
