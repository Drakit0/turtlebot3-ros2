from enum import Enum, auto
import numpy as np


class WallFollowerStates(Enum):
    STOP = auto()
    FORWARD = auto()
    RIGHT = auto()
    LEFT = auto()
    TURN180 = auto()
    RIGHT_PATH = auto()


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float) -> None:
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt
        self._state = WallFollowerStates.STOP
        self.min_dist = 0.25
        self.current_min_dist = self.min_dist
        self.reference_distance = 0.2

        self.Kp = 2
        self.Kd = 1.5
        self.last_error = None

    def control(
        self, z_scan: list[float], z_v: float, z_w: float
    ) -> tuple[float, float]:
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
        v = 0.1

        index = -(len(z_scan) // 4)
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

        return v, w

    def compute_commands(
        self, z_scan: list[float], z_v: float, z_w: float
    ) -> tuple[float, float]:
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
        n = len(z_scan)
        front_close = d_front < self.current_min_dist
        d_back = z_scan[n//2]
        d_right = z_scan[-(n // 4)]
        d_right_45 = z_scan[-(n // 8)]
        d_left = z_scan[n // 4]
        d_left_45 = z_scan[n // 8]
        d_left_middle = z_scan[n // 8 + n // 16]
        d_right_middle = z_scan[-(n // 8) - (n // 16)]
        d_right_45_back = z_scan[-(n // 8) - (n // 4)]
        d_left_45_back = z_scan[n // 8 + (n // 4)]
        d_right_middle_back = z_scan[-(n//4) - (n//16)]
        d_left_middle_back = z_scan[n//4 + (n//16)]
        

        # Transitions
        if self._state is WallFollowerStates.STOP:
            self._state = WallFollowerStates.FORWARD

        elif self._state is WallFollowerStates.FORWARD:
            if front_close:
                if d_right > self.current_min_dist or d_left > self.current_min_dist:
                    if d_right > d_left:
                        self._state = WallFollowerStates.RIGHT
                    else:
                        self._state = WallFollowerStates.LEFT
                else:
                    self._state = WallFollowerStates.TURN180
            else:
                # Si hay hueco a la derecha el robot gira a la derecha
                if d_right > 3*self.current_min_dist and d_back > 1.5*self.current_min_dist:# and d_right_middle > self.current_min_dist and d_right_middle_back > self.current_min_dist:
                    self._state = WallFollowerStates.RIGHT_PATH
                

        elif self._state is WallFollowerStates.RIGHT:
            # Comprueba si hay hueco delante y si el rayo a 90º, 45º y 22.5º siguen los ratios que deben seguir para que la pared esté paralela
            if (
                not front_close
                # and (np.isclose(d_left * np.sqrt(2), d_left_45, atol=0.01)
                and np.isclose(d_left, 0.923879 * d_left_middle, atol=0.01)
                # or np.isclose(d_left*np.sqrt(2), d_left_45_back, atol=0.01)
                and np.isclose(d_left, 0.923879 * d_left_middle_back, atol=0.01)             
            ):
                self._state = WallFollowerStates.FORWARD
                
        elif self._state is WallFollowerStates.RIGHT_PATH:
            if d_front >= 2*self.current_min_dist and np.isclose(d_left * np.sqrt(2), d_left_45, atol=0.02) and np.isclose(d_left, 0.923879 * d_left_middle, atol=0.01):#(d_right <= self.current_min_dist or d_right_middle <= self.current_min_dist) and (d_left <= self.current_min_dist or d_left_middle <= self.current_min_dist):
                self._state = WallFollowerStates.FORWARD
            elif d_front <= self.current_min_dist:
                self._state = WallFollowerStates.LEFT

        elif self._state is WallFollowerStates.LEFT:
            # np.cos(np.pi/8) = 0.9238795325112867
            # Comprueba si hay hueco delante y si el rayo a 90º, 45º y 22.5º siguen los ratios que deben seguir para que la pared esté paralela
            if (
                not front_close
                # and (np.isclose(d_right * np.sqrt(2), d_right_45, atol=0.01)
                and np.isclose(d_right, 0.923879 * d_right_middle, atol=0.02)
                # or np.isclose(d_right*np.sqrt(2), d_right_45_back, atol=0.01)
                and np.isclose(d_right, 0.923879 * d_right_middle_back, atol=0.01)
            ):
                self._state = WallFollowerStates.FORWARD

        elif self._state is WallFollowerStates.TURN180:
            if (
                not front_close
                and np.isclose(d_right * np.sqrt(2), d_right_45, atol=0.01)
                and np.isclose(d_right, 0.923879 * d_right_middle, atol=0.02)
            ):
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
            
        elif self._state is WallFollowerStates.RIGHT_PATH:
            # self.current_min_dist = 0.5
            v = 0.05
            w = -v / 0.2

        elif self._state is WallFollowerStates.LEFT:
            # self.current_min_dist = 0.5
            w = 0.3
            v = 0.0

        elif self._state is WallFollowerStates.TURN180:
            w = 0.3
            v = 0.0
        return v, w
