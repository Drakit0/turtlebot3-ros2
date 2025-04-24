import numpy as np
from sensor_msgs.msg import LaserScan


def clip_angle(angle: float) -> float:
    """
    Clip the angle to the range [-pi, pi].
    """

    return (angle + np.pi) % (2 * np.pi) - np.pi


class EKF:
    def __init__(
        self,
        dt: float,
        initial_pose: tuple[float, float, float] = (
            float("nan"),
            float("nan"),
            float("nan"),
        ),
        initial_covariance: tuple[float, float, float] = (
            float("nan"),
            float("nan"),
            float("nan"),
        ),
        sigma_v: float = 0.05,
        sigma_w: float = 0.1,
        sigma_z: float = 0.2,
        wall_params: list[tuple[float, float]] = [
            (0.5, 0.5),  # Wall 1: (x, y)
        ],
    ):
        self.mu = np.array(initial_pose)
        self.Sigma = np.diag(np.array(initial_covariance))
        self.dt = dt
        self.sv2 = sigma_v**2
        self.sw2 = sigma_w**2
        self.sz2 = sigma_z**2
        self.wall_params = wall_params  # List of tuples (x, y) for wall positions
        self.maha_thres = 3.8

    def predict(self, v: float, w: float):
        """
        Predict the next state of the robot using the motion model.
        It also updates the noise covariance.

        Args
        ----
         - v: linear velocity of the robot
         - w: angular velocity of the robot

        """
        x = self.mu[0]
        y = self.mu[1]
        theta = self.mu[2]
        # R_t = 0.001 * np.eye(3)

        dt = self.dt

        # No rotation
        if abs(w) < 1e-5:
            g_x = x + v * dt * np.cos(theta)
            g_y = y + v * dt * np.sin(theta)
            g_theta = theta

        else:
            g_x = x + (-v / w * np.sin(theta) + v / w * np.sin(theta + w * dt))
            g_y = y + (v / w * np.cos(theta) - v / w * np.cos(theta + w * dt))
            g_theta = theta + w * dt

        # Predict Jacobian motion
        if abs(w) < 1e-5:
            G = np.array(
                [[1, 0, -v * dt * np.sin(theta)], [0, 1, v * dt * np.cos(theta)], [0, 0, 1]]
            )

        else:
            G = np.array(
                [
                    [1, 0, -v / w * np.cos(theta) + v / w * np.cos(theta + w * dt)],
                    [0, 1, -v / w * np.sin(theta) + v / w * np.sin(theta + w * dt)],
                    [0, 0, 1],
                ]
            )

        # Predict Jacobian noise
        if abs(w) < 1e-5:
            V = np.array(
                [
                    [dt * np.cos(theta), 0],
                    [dt * np.sin(theta), 0],
                    [0, dt],
                ]
            )

        else:
            V = np.array(
                [
                    [
                        (-np.sin(theta) + np.sin(theta + w * dt)) / w,
                        v * (np.sin(theta) - np.sin(theta + w * dt)) / (w**2)
                        + v * dt * np.cos(theta + w * dt) / w,
                    ],
                    [
                        (np.cos(theta) - np.cos(theta + w * dt)) / w,
                        -v * (np.cos(theta) - np.cos(theta + w * dt)) / (w**2)
                        + v * dt * np.sin(theta + w * dt) / w,
                    ],
                    [0, dt],
                ]
            )

        # Moving noise cov
        R_t = V @ np.diag([self.sv2, self.sw2]) @ V.T

        # Update moving state
        self.mu = np.array([g_x, g_y, clip_angle(g_theta)])
        self.Sigma = G @ self.Sigma @ G.T + R_t

        # return self.mu

    def update(self, scan: LaserScan):
        """
        Update the state of the robot using the laser scan measurements.

        Args
        ----
         - z_scan: Laser scan measurements
        """

        # Extract walls meassurements
        z_scan = np.array(scan.ranges)
        min_angle = 0
        inc_angle = 2 * np.pi / len(z_scan)
        angles = min_angle + np.arange(len(z_scan)) * inc_angle

        r_right, phi_right = self.avg_measurements(
            z_scan,
            min_angle,
            inc_angle,
            angles,
            -np.pi / 2,
        )
        r_left, phi_left = self.avg_measurements(
            z_scan,
            min_angle,
            inc_angle,
            angles,
            np.pi / 2,
        )

        # Associate walls
        (alpha_right, rho_right), (alpha_left, rho_left) = self.select_wall()

        measurements = []

        if r_right is not None:
            measurements.append((r_right, phi_right, alpha_right, rho_right))
        if r_left is not None:
            measurements.append((r_left, phi_left, alpha_left, rho_left))

        # No walls detected (corner)
        if len(measurements) == 0:
            return self.mu

        # Updating noise cov
        Q_t = np.diag([self.sz2, self.sz2])

        # Update based on landmarks
        for r, phi, alpha, rho in measurements:
            x = self.mu[0]
            y = self.mu[1]
            theta = self.mu[2]

            rho_pred = rho - (x * np.cos(alpha) + y * np.sin(alpha))
            gamma_pred = clip_angle(alpha - theta)

            # Update Jacobian
            H = np.array(
                [
                    [-np.cos(alpha), -np.sin(alpha), 0],
                    [0, 0, -1],
                ]
            )

            # Innovation vec: diff between pred and meas
            y_vec = np.array([r - rho_pred, clip_angle(phi - gamma_pred)])

            # Innovation cov
            S = H @ self.Sigma @ H.T + Q_t

            # Mahalanobis gate in case of outliers: dist point to prob dist
            if float(y_vec.T @ np.linalg.inv(S) @ y_vec) > self.maha_thres:
                continue

            K = self.Sigma @ H.T @ np.linalg.inv(S)
            self.mu += K @ y_vec
            self.mu[2] = clip_angle(self.mu[2])
            self.Sigma = (np.eye(3) - K @ H) @ self.Sigma

        return self.mu

    def avg_measurements(
        self,
        z_scan: LaserScan.ranges,
        min_angle: float,
        inc_angle: float,
        angles: np.ndarray,
        center_angle: float,
        window=5,
    ):
        """
        Average the measurements in a window around the given angle.
        """

        # Find closest laser scan index to the center angle
        center_index = int((center_angle - min_angle) / inc_angle)

        if center_index < 0:
            center_index = len(z_scan) + center_index

        # indices = np.clip(
        #     np.arange(center_index - window, center_index + window + 1), 0, len(z_scan) - 1
        # )

        # Find rs and thetas
        rs = z_scan[center_index - window : center_index + window + 1]
        thetas = angles[center_index - window : center_index + window + 1]

        # pick the beam whose angle is closest to center_angle
        diffs = clip_angle(angles - center_angle)
        center_index = int(np.argmin(np.abs(diffs)))

        # wrap around if necessary
        idxs = np.arange(center_index - window, center_index + window + 1) % len(z_scan)
        rs = z_scan[idxs]
        thetas = angles[idxs]

        # Check rs
        valid = np.isfinite(rs)
        if not np.any(valid):
            return None, None

        r = np.mean(rs[valid])
        phi = clip_angle(np.mean(thetas[valid]))

        return r, phi

    def select_wall(self):
        """
        Select the left and right walls based on the robot's current orientation.
        """

        _, _, theta = self.mu
        bearings = [clip_angle(alpha - theta) for alpha, _ in self.wall_params]
        idx_r = np.argmin([np.abs(b + np.pi / 2) for b in bearings])
        idx_l = np.argmin([np.abs(b - np.pi / 2) for b in bearings])

        return self.wall_params[idx_r], self.wall_params[idx_l]
