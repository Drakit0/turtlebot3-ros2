import numpy as np


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
    ):
        self.mu = np.array(initial_pose)
        self.Sigma = np.diag(np.array(initial_covariance))
        self.dt = dt
        self.sigma_v = sigma_v
        self.sigma_w = sigma_w
        self.sigma_z = sigma_z

    def predict(self, v, w):
        x = self.mu[0]
        y = self.mu[1]
        theta = self.mu[2]
        R_t = 0.001 * np.eye(3)

        dt = self.dt

        g_x = x + (-v / w * np.sin(theta) + v / w * np.sin(theta + w * dt))
        g_y = y + (v / w * np.cos(theta) - v / w * np.cos(theta + w * dt))
        g_theta = theta + w * dt

        self.mu[0] = g_x
        self.mu[1] = g_y
        self.mu[2] = g_theta
        self.mu[2] %= 2 * np.pi

        G = np.array(
            [
                [
                    1,
                    0,
                    -v / w * np.cos(self.mu[2]) + v / w * np.cos(self.mu[2] + w * dt),
                ],
                [
                    0,
                    1,
                    -v / w * np.sin(self.mu[2]) + v / w * np.sin(self.mu[2] + w * dt),
                ],
                [0, 0, 1],
            ]
        )

        self.Sigma = G @ self.Sigma @ G.T + R_t

    def update(self, z_scan, pose):
        Q_t = 0.001 * np.eye(2)
        d_alpha = 2 * np.pi / len(z_scan)

        for i, r in enumerate(z_scan):
            phi = i * d_alpha

            dx_r = r * np.cos(phi) / np.sqrt(r)
            dy_r = r * np.sin(phi) / np.sqrt(r)

            dx = dx_r - (pose[0] - self.mu[0])
            dy = dy_r - (pose[1] - self.mu[1])
            d = (dx**2 + dy**2) ** (1 / 2)

            H = np.array(
                [
                    [-dx / d ** (1 / 2), -dy / d ** (1 / 2), 0],
                    [dy / d, -dx / d, -1],
                ]
            )
            S = H @ self.Sigma @ H.T + Q_t
            K = self.Sigma @ H.T @ np.linalg.inv(S)
            y = np.array([z_scan[i], phi]) - np.array(
                [d, np.arctan2(dy, dx) - self.mu[2]]
            )
            self.mu += K @ y
            self.mu[2] %= 2 * np.pi
            self.Sigma = (np.eye(3) - K @ H) @ self.Sigma
        return self.mu
