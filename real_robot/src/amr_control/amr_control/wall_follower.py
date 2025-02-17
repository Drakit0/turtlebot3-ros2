from enum import Enum, auto
import math
import numpy as np

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
        self.Kp = 6 # 2
        self.K = 10
        self.Ti = 0.5
        self.Td = 3
        self.last_measurement_wall_distance = None
        self.integral = 0.0
        self.min_dist = 0.25
        self.current_min_dist = self.min_dist
        self.reference_distance = 0.2
        
    def control(self, z_scan: list[float], z_v:float, z_w: float) -> tuple[float, float]:
        v = 0.1
        w = 0.0
        n = len(z_scan) // 4
        n2 = 7
        d1 = z_scan[-n]
        d2 = z_scan[-n+n2]
        
        # Get measurements
        angle = 2 * np.pi * n2 / len(z_scan) # Angle between the rays
        theta = -math.atan((d1 - d2*np.cos(angle))/(d2*np.sin(angle))) # Angle with the wall
        distance = np.cos(theta) * d1 # Distance to the wall
        
        # Calculate outerloop error
        error_outerloop = self.reference_distance - distance
        
        # Add to integral part
        self.integral += error_outerloop * self._dt
        
        # Derivative of measured distance to wall
        derivative = (distance - self.last_measurement_wall_distance) / self._dt if self.last_measurement_wall_distance is not None else 0.0
        self.last_measurement_wall_distance = distance
        derivative = 0.0
        # Outerloop control
        reference_angle = self.K * error_outerloop + self.integral / self.Ti + self.Td * derivative
        
        # Calculate innerloop error
        error = reference_angle - theta
        
        # Innerloop Control
        w = self.Kp * error
        return v, w
        
    def compute_commands(self, z_scan: list[float], z_v: float, z_w: float) -> tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_scan: Distance from every LiDAR ray to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        # TODO: 2.14. Complete the function body with your code (i.e., compute v and w).
        n_points = 3

        # get measurements
        n = len(z_scan) // 4
        n_45 = len(z_scan) // 8
        front = min(z_scan[-n_points:] + z_scan[:n_points])
        front_close = front < self.current_min_dist
        left = min(z_scan[n-n_points: n+n_points])
        right = min(z_scan[-n-n_points: -n+n_points])
        left_close = left < self.current_min_dist
        right_close = right < self.current_min_dist
        right_45 = min(z_scan[n_45-n_points: n_45+n_points])
        left_45 = min(z_scan[n_45-n-n_points: n_45-n+n_points])

        
        # Transitions
        if self._state is WallFollowerStates.STOP:
            self._state = WallFollowerStates.FORWARD
        elif self._state is WallFollowerStates.FORWARD:

            if front_close:
                if left_close and right_close:
                    self._state = WallFollowerStates.TURN180
                elif left < right:
                    self._state = WallFollowerStates.RIGHT
                else:
                    self._state = WallFollowerStates.LEFT
                    
        elif self._state is WallFollowerStates.RIGHT:
            if front >= 0.4 and left_45 > 0.20 and right_45 > 0.20:
                self.integral = 0
                self._state = WallFollowerStates.FORWARD
                
        elif self._state is WallFollowerStates.LEFT:
            if front >= 0.4 and left_45 > 0.20 and right_45 > 0.20:
                self.integral = 0
                self._state = WallFollowerStates.FORWARD
                
        elif self._state is WallFollowerStates.TURN180:
            forward_free = front > 0.4 and left_45 > 0.20 and right_45 > 0.20
            if forward_free:
                self.integral = 0
                self._state = WallFollowerStates.FORWARD

        # Actions
        if self._state is WallFollowerStates.STOP:
            v = 0.0
            w = 0.0
        elif self._state is WallFollowerStates.FORWARD:
            # self.current_min_dist = 0.25
            v, w = self.control(z_scan, z_v, z_w)
                    
        elif self._state is WallFollowerStates.RIGHT:
            # self.current_min_dist = 0.5
            w = -0.3
            if front_close:
                v = 0.0
            else:
                v = 0.05
        elif self._state is WallFollowerStates.LEFT:
            # self.current_min_dist = 0.5
            w = 0.3
            if front_close:
                v = 0.0
            else:
                v = 0.05
                
        elif self._state is WallFollowerStates.TURN180:
            w = 0.3
            v = 0.0

        return v, w
