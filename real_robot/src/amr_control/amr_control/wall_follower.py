from enum import Enum, auto
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
        
        index = - (len(z_scan) // 4)
        measured_distance = z_scan[index]
        measured_distance_left = z_scan[-index]
        
        if measured_distance_left < measured_distance + 0.1:
            error = measured_distance_left - self.reference_distance
        else:
            error = self.reference_distance - measured_distance
        
        # First derivative run is 0
        if self.last_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.last_error) / self._dt
        
        self.last_error = error
        w = self.Kp * error + self.Kd * derivative
        
        # Clip the angular velocity command to the range [-1, 1].
        w = np.clip(w, -1.0, 1.0)
        
        # rclpy.logging.get_logger("WallFollower").warn(
        #     f"PD Control -> measured_distance: {measured_distance:.04f}, error: {error:.04f}, "
        #     f"derivative: {derivative:.04f}, w: {w:.04f}"
        # )
        
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
        
        # get measurements
        
        d_front = z_scan[0]
        front_close = d_front < self.current_min_dist
        d_right = z_scan[-(len(z_scan) // 4)]
        d_right_45 = z_scan[-(len(z_scan) // 8)]
        d_left = z_scan[len(z_scan) // 4]
        d_left_45 = z_scan[len(z_scan) // 8]
        
        rclpy.logging.get_logger("state").warn(
            f"state {self._state}"
        )

        
        # Transitions
        if self._state is WallFollowerStates.STOP:
            self._state = WallFollowerStates.FORWARD
            
        elif self._state is WallFollowerStates.FORWARD:
            # rclpy.logging.get_logger("d1, d2").warn(
            #     f"d1: {d1:.03f}, d2: {d2:.03f}"
            # )   
            
            if front_close and d_right > self.current_min_dist and d_right > d_left:
                self._state = WallFollowerStates.RIGHT
                
            elif front_close and d_left > self.current_min_dist and d_left > d_right:
                self._state = WallFollowerStates.LEFT
                
            elif front_close and d_right < self.current_min_dist and d_left < self.current_min_dist:
                self._state = WallFollowerStates.TURN180
                    
        elif self._state is WallFollowerStates.RIGHT:
            rclpy.logging.get_logger("d_right, d_right_45").warn(
                f"d_right: {d_left:.03f}, d_right_45: {d_left_45:.03f}"
            )
            if not front_close and np.isclose(d_left * np.sqrt(2), d_left_45, atol=0.01):
                self._state = WallFollowerStates.FORWARD
                
        elif self._state is WallFollowerStates.LEFT:
            rclpy.logging.get_logger("d_left, d_left_45").warn(
                f"d_left: {d_right:.03f}, d_left_45: {d_right_45:.03f}, sqrt2:{d_right * np.sqrt(2):.03f}, close: {np.isclose(d_right * np.sqrt(2), d_right_45, atol=0.02)}"
            )
            if not front_close and np.isclose(d_right * np.sqrt(2), d_right_45, atol=0.01):
                self._state = WallFollowerStates.FORWARD
                
        elif self._state is WallFollowerStates.TURN180:
            if not front_close and np.isclose(d_right * np.sqrt(2), d_right_45, atol=0.01):
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
            v = 0.0
                
        elif self._state is WallFollowerStates.LEFT:
            # self.current_min_dist = 0.5
            w = 0.3
            v = 0.0
                
        elif self._state is WallFollowerStates.TURN180:
            w = 0.3
            v = 0.0

        return v, w
