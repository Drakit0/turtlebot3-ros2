from enum import Enum, auto
import math
import numpy as np
import rclpy

class WallFollowerStates(Enum):
    STOP = auto()
    FOLLOW = auto()
    FORWARD = auto()
    RIGHT = auto()
    LEFT = auto()
    TURN180 = auto()

class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""
    
    def __init__(self, dt: float) -> None:
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt
        self._state = WallFollowerStates.STOP 
        self.Kp = 0.6 # 2
        self.K = 5
        self.Ti = 1
        self.Td = 3
        self.last_measurement_wall_distance = None
        self.last_measurement_wall_distance2 = None
        self.integral = 0.0
        self.min_dist = 0.25
        self.current_min_dist = self.min_dist
        self.reference_distance = 0.2
        
        
        self.Kp = 2
        self.Kd = 1.5
        self.last_error = None
        
    # def control(self, z_scan: list[float], z_v:float, z_w: float) -> tuple[float, float]:
    #     v = 0.1
    #     w = 0.0
    #     n = len(z_scan) // 4
    #     n2 = 7
    #     d1 = z_scan[-n]
    #     d2 = z_scan[-n+n2]
        
    #     # Get measurements
    #     angle = 2 * np.pi * n2 / len(z_scan) # Angle between the rays
    #     theta = -math.atan((d1 - d2*np.cos(angle))/(d2*np.sin(angle))) # Angle with the wall
    #     distance = np.cos(theta) * d1 # Distance to the wall
        
    #     # Calculate outerloop error
    #     error_outerloop = self.reference_distance - distance
        
    #     # Add to integral part
    #     self.integral += error_outerloop * self._dt
        
    #     # Derivative of measured distance to wall
    #     derivative = (error_outerloop - self.last_measurement_wall_distance) / self._dt if self.last_measurement_wall_distance is not None else 0.0
    #     self.last_measurement_wall_distance = error_outerloop
    #     derivative = 0.0
    #     # Outerloop control
    #     reference_angle = self.K * error_outerloop + self.integral / self.Ti + self.Td * derivative
        
    #     # # Calculate innerloop error
    #     # error = reference_angle - theta
        
    #     # # Innerloop Control
    #     # w = self.Kp * error
    #     w = reference_angle
        
    #     w = np.clip(w, -1.0, 1.0)
        
    #     # derivative = (self.last_measurement_wall_distance - self.last_measurement_wall_distance2) / self._dt if self.last_measurement_wall_distance2 is not None else 0.0
    #     # w = 5 * (self.reference_distance - d1) + 4 * derivative
    #     # self.last_measurement_wall_distance2 = self.last_measurement_wall_distance
    #     # self.last_measurement_wall_distance = w
        
    #     # rclpy.logging.get_logger("Distancias").warn(
    #     #     f"D1: {d1:.04f}, D2: {d2:.02f}, distance: {distance:.02f}, theta: {theta:.02f}"
    #     # )
    #     rclpy.logging.get_logger("mando").warn(
    #         f"Mando: {w:.04f}, error seguimiento: {self.reference_distance - d1:.04f}"
    #     )
    #     return v, w
    
    def control(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """
        PD controller for wall following.
        
        This controller computes the error between the desired and the
        measured distance to the wall and then applies a PD law to produce
        an angular velocity command.
        
        Args:
            z_scan: List of LiDAR distance measurements [m].
            z_v: Current linear velocity (not used here).
            z_w: Current angular velocity (not used here).
            
        Returns:
            v: Linear velocity command [m/s].
            w: Angular velocity command [rad/s].
        """
        # Forward speed
        v = 0.15
        
        #  Here we choose an index from the last quarter of the scan.)
        index = len(z_scan) - (len(z_scan) // 4)
        measured_distance = z_scan[index]
        
        # --- Compute error and its derivative ---
        # Error is defined as the difference between the desired and measured distance.
        error = self.reference_distance - measured_distance
        
        # Compute the derivative of the error. On the first run, set derivative to zero.
        if self.last_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.last_error) / self._dt
        
        # Save the error for the next iteration
        self.last_error = error
        
        # --- Compute angular velocity using the PD law ---
        w = self.Kp * error + self.Kd * derivative
        
        # Optionally, clip w to a safe range (e.g., -1 to 1 rad/s)
        w = np.clip(w, -1.0, 1.0)
        
        rclpy.logging.get_logger("WallFollower").warn(
            f"PD Control -> measured_distance: {measured_distance:.04f}, error: {error:.04f}, "
            f"derivative: {derivative:.04f}, w: {w:.04f}"
        )
        
        return v, w
        
    # def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
    #     """Wall following exploration algorithm.

    #     Args:
    #         z_scan: Distance from every LiDAR ray to the closest obstacle [m].
    #         z_v: Odometric estimate of the linear velocity of the robot center [m/s].
    #         z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

    #     Returns:
    #         v: Linear velocity [m/s].
    #         w: Angular velocity [rad/s].

    #     """
    #     # TODO: 2.14. Complete the function body with your code (i.e., compute v and w).
    #     n_points = 3

    #     # get measurements
    #     n = len(z_scan) // 4
    #     n_45 = len(z_scan) // 8
    #     front = min(z_scan[-n_points:] + z_scan[:n_points])
    #     front_close = front < self.current_min_dist
    #     left = min(z_scan[n-n_points: n+n_points])
    #     right = min(z_scan[-n-n_points: -n+n_points])
    #     left_close = left < self.current_min_dist
    #     right_close = right < self.current_min_dist
    #     right_45 = min(z_scan[n_45-n_points: n_45+n_points])
    #     left_45 = min(z_scan[n_45-n-n_points: n_45-n+n_points])

    #     n2 = 7
    #     d1 = z_scan[-n]
    #     d2 = z_scan[-n+n2]
    #     d1l = z_scan[n]
    #     d2l = z_scan[n-n2]
    #     rclpy.logging.get_logger("state").warn(
    #         f"state {self._state}"
    #     )

        
    #     # Transitions
    #     if self._state is WallFollowerStates.STOP:
    #         self._state = WallFollowerStates.FORWARD
    #     # elif self._state is WallFollowerStates.FORWARD:
    #     #     # rclpy.logging.get_logger("d1, d2").warn(
    #     #     #     f"d1: {d1:.03f}, d2: {d2:.03f}"
    #     #     # )       
    #     #     if d2 > 1 and d1 < 0.3:
    #     #         self._state = WallFollowerStates.RIGHT
    #     #     elif d2l > 1 and d1l < 0.3:
    #     #         self._state = WallFollowerStates.LEFT
    #     #     elif front_close:
    #     #         if left_close and right_close:
    #     #             self._state = WallFollowerStates.TURN180
    #     #         elif left < right:
    #     #             self._state = WallFollowerStates.RIGHT
    #     #         else:
    #     #             self._state = WallFollowerStates.LEFT
                    
    #     # elif self._state is WallFollowerStates.RIGHT:
    #     #     if front >= 0.5 and left_45 > 0.3 and right_45 > 0.3:
    #     #         self.integral = 0
    #     #         self._state = WallFollowerStates.FORWARD
                
    #     # elif self._state is WallFollowerStates.LEFT:
    #     #     if front >= 0.5 and left_45 > 0.3 and right_45 > 0.3:
    #     #         self.integral = 0
    #     #         self._state = WallFollowerStates.FORWARD
                
    #     # elif self._state is WallFollowerStates.TURN180:
    #     #     forward_free = front > 0.4 and left_45 > 0.20 and right_45 > 0.20
    #     #     if forward_free:
    #     #         self.integral = 0
    #     #         self._state = WallFollowerStates.FORWARD

    #     # Actions
    #     if self._state is WallFollowerStates.STOP:
    #         v = 0.0
    #         w = 0.0
    #     elif self._state is WallFollowerStates.FORWARD:
    #         # self.current_min_dist = 0.25
    #         v, w = self.control(z_scan, z_v, z_w)
                    
    #     elif self._state is WallFollowerStates.RIGHT:
    #         # self.current_min_dist = 0.5
    #         w = -0.3
    #         if front_close:
    #             v = 0.0
    #         else:
    #             v = 0.05
    #     elif self._state is WallFollowerStates.LEFT:
    #         # self.current_min_dist = 0.5
    #         w = 0.3
    #         if front_close:
    #             v = 0.0
    #         else:
    #             v = 0.05
                
    #     elif self._state is WallFollowerStates.TURN180:
    #         w = 0.3
    #         v = 0.0

    #     return v, w
    
    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """
        Wall following exploration algorithm with improved handling on curves.
        """
        # Indices selection for wall measurements:
        # Assume the wall is on the robot's right side.
        n = len(z_scan) // 4  # roughly right side
        n_offset = 7          # offset to get a second point further ahead
        
        # Compute averaged distances for more robust measurement.
        d1 = self.compute_wall_estimate(z_scan, len(z_scan) - n, window=3)
        d2 = self.compute_wall_estimate(z_scan, len(z_scan) - n + n_offset, window=3)
        
        # Calculate the angle between the two measurements.
        # Determine the angular spacing between rays.
        angle_resolution = 2 * math.pi / len(z_scan)
        delta_angle = n_offset * angle_resolution  # approximate angular difference
        
        # Using geometry, compute the wall angle relative to the robot.
        # This equation assumes that the wall is straight between the two measured points.
        # If the wall is curving, this gives an estimate of the local orientation.
        # (Be sure to check for division by zero.)
        if d2 - d1 * math.cos(delta_angle) != 0:
            theta = math.atan2(d1 * math.sin(delta_angle), d2 - d1 * math.cos(delta_angle))
        else:
            theta = 0.0
        
        # Compute the errors:
        distance_error = self.reference_distance - d1
        # For the angle, we want the wall to be parallel to the robot, so desired theta = 0.
        angle_error = -theta  # negative sign to steer toward aligning with the wall

        # Derivative calculations for smoothing (if you want derivative control):
        if self.last_distance_error is None:
            d_error = 0.0
        else:
            d_error = (distance_error - self.last_distance_error) / self._dt
        self.last_distance_error = distance_error
        
        if self.last_angle_error is None:
            a_error = 0.0
        else:
            a_error = (angle_error - self.last_angle_error) / self._dt
        self.last_angle_error = angle_error
        
        # Compute control actions using a PD law on both errors:
        # Angular control is a sum of corrections from both distance and angle errors.
        w_distance = self.Kp_distance * distance_error + self.Kd_distance * d_error
        w_angle = self.Kp_angle * angle_error + self.Kd_angle * a_error
        
        # Combine the contributions.
        w = w_distance + w_angle
        
        # Optionally, reduce forward speed on sharp curves:
        if abs(theta) > 0.3:  # if the wall angle is large, we're on a curve
            v = 0.05
        else:
            v = 0.1
        
        # Optionally, you can add additional state-based behavior here for obstacle avoidance:
        n_points = 3
        front = min(z_scan[-n_points:] + z_scan[:n_points])
        if front < self.current_min_dist:
            # If something is very close in front, slow down further.
            v = 0.0
            # You could also set w to a fixed value to turn.
        
        # Log values for debugging:
        rclpy.logging.get_logger("state").warn(
            f"d1: {d1:.03f}, d2: {d2:.03f}, theta: {theta:.03f}, dist_error: {distance_error:.03f}, angle_error: {angle_error:.03f}, v: {v:.03f}, w: {w:.03f}"
        )
        
        return v, w
