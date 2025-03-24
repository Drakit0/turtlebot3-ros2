import datetime
import math
import numpy as np
import os
import pytz
import random
import rclpy

from amr_localization.maps import Map
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN

import rclpy.logging


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(
        self,
        dt: float,
        map_path: str,
        particle_count: int,
        sigma_v: float = 0.05,
        sigma_w: float = 0.1,
        sigma_z: float = 0.2,
        sensor_range_max: float = 1.0,  # cambiado de 8.0 a 1.0
        sensor_range_min: float = 0.16,
        global_localization: bool = True,
        initial_pose: tuple[float, float, float] = (
            float("nan"),
            float("nan"),
            float("nan"),
        ),
        initial_pose_sigma: tuple[float, float, float] = (
            float("nan"),
            float("nan"),
            float("nan"),
        ),
    ):
        """Particle filter class initializer.

        Args:
            dt: Sampling period [s].
            map_path: Path to the map of the environment.
            particle_count: Initial number of particles.
            sigma_v: Standard deviation of the linear velocity [m/s].
            sigma_w: Standard deviation of the angular velocity [rad/s].
            sigma_z: Standard deviation of the measurements [m].
            sensor_range_max: Maximum sensor measurement range [m].
            sensor_range_min: Minimum sensor measurement range [m].
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].

        """
        self._dt: float = dt
        self._initial_particle_count: int = particle_count
        self._particle_count: int = particle_count
        self._sensor_range_max: float = sensor_range_max
        self._sensor_range_min: float = sensor_range_min
        self._sigma_v: float = sigma_v
        self._sigma_w: float = sigma_w
        self._sigma_z: float = sigma_z
        self._iteration: int = 0
        self._initial_pose_sigma: tuple[float, float, float] = initial_pose_sigma

        self._map = Map(
            map_path,
            sensor_range_max,
            compiled_intersect=True,
            use_regions=True,  # Cambiado de False a True
            safety_distance=0.08,
        )
        self._particles = self._init_particles(
            particle_count, global_localization, initial_pose, initial_pose_sigma
        )
        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(
            pytz.timezone("Europe/Madrid")
        ).strftime("%Y-%m-%d_%H-%M-%S")

        self._num_rays = 8

    def compute_pose(self) -> tuple[bool, tuple[float, float, float]]:
        """Computes the pose estimate when the particles form a single DBSCAN cluster.

        Adapts the amount of particles depending on the number of clusters during localization.
        100 particles are kept for pose tracking.

        Returns:
            localized: True if the pose estimate is valid.
            pose: Robot pose estimate (x, y, theta) [m, m, rad].

        """
        # TODO: 3.10. Complete the missing function body with your code.
        localized: bool = False
        pose: tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))

        # Optimize DBSCAN parameters for better performance
        dbscan = DBSCAN(eps=0.1, min_samples=10, algorithm="kd_tree", n_jobs=-1)

        # Avoid unnecessary modulo operations by normalizing angles only when needed
        cos_theta = np.cos(self._particles[:, 2].astype(np.float32))
        sin_theta = np.sin(self._particles[:, 2].astype(np.float32))

        # Create features array directly without intermediate steps
        features = np.column_stack(
            (self._particles[:, 0], self._particles[:, 1], cos_theta, sin_theta)
        )

        # Perform clustering
        labels = dbscan.fit_predict(features)

        # Check if only one cluster (excluding noise points)
        valid_labels = labels[labels != -1]
        localized = len(np.unique(valid_labels)) == 1 and len(valid_labels) > 0

        if localized:
            # # Reduce particles for tracking mode
            # self._particle_count = 50
            # # Calculate mean pose directly
            # mean_x = np.mean(self._particles[:, 0])
            # mean_y = np.mean(self._particles[:, 1])
            # mean_theta = np.mean(self._particles[:, 2])

            # pose = (mean_x, mean_y, mean_theta)
            # self._particles = self._init_particles(
            #     self._particle_count, False, pose, (0.1, 0.1, math.radians(5))
            # )
            # Reduce particles for tracking mode
            self._particle_count = 50
            # Use more efficient random sampling
            particle_idx = np.random.choice(self._particles.shape[0], self._particle_count, replace=False)
            self._particles = self._particles[particle_idx]
            # Calculate mean pose directly
            mean_x = np.mean(self._particles[:, 0])
            mean_y = np.mean(self._particles[:, 1])
            # Calculate mean angle properly (average of unit vectors)
            mean_theta = np.arctan2(np.mean(sin_theta[particle_idx]), np.mean(cos_theta[particle_idx]))
            pose = (mean_x, mean_y, mean_theta)

        return localized, pose

    def move(self, v: float, w: float) -> None:
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m].
            w: Angular velocity [rad/s].

        """
        self._iteration += 1

        # TODO: 3.5. Complete the function body with your code.
        v_with_noise = v + np.random.normal(0, self._sigma_v, self._particle_count)
        w_with_noise = w + np.random.normal(0, self._sigma_w, self._particle_count)

        x_new = (
            self._particles[:, 0]
            + v_with_noise * np.cos(self._particles[:, 2].astype(np.float32)) * self._dt
        )
        y_new = (
            self._particles[:, 1]
            + v_with_noise * np.sin(self._particles[:, 2].astype(np.float32)) * self._dt
        )
        theta_new = (self._particles[:, 2] + w_with_noise * self._dt) % (2 * np.pi)
        for i, (x, y) in enumerate(zip(x_new, y_new)):
            if all((x, y) != self._particles[i, :2]):
                intersection, _ = self._map.check_collision(
                    [(x, y), self._particles[i, :2]]
                )
                if intersection:
                    x_new[i] = intersection[0]
                    y_new[i] = intersection[1]

        self._particles[:, 0] = x_new
        self._particles[:, 1] = y_new
        self._particles[:, 2] = theta_new

    def resample(self, measurements: list[float]) -> None:
        """Samples a new set of particles.

        Args:
            measurements: Sensor measurements [m].

        """
        # TODO: 3.9. Complete the function body with your code (i.e., replace the pass statement).
        weights = [
            self._measurement_probability(measurements, particle)
            for particle in self._particles
        ]
        weights /= np.sum(weights)

        cumulative_weights = np.cumsum(weights)
        strata_boundaries = np.linspace(0, 1, self._particle_count + 1)

        resampled_particles = np.zeros_like(self._particles)

        for i in range(self._particle_count):
            random_sample = np.random.uniform(
                strata_boundaries[i], strata_boundaries[i + 1]
            )
            index = np.searchsorted(cumulative_weights, random_sample)
            resampled_particles[i] = self._particles[index]

        self._particles = resampled_particles
        return
        similarities = np.array(
            [
                self._measurement_probability(measurements, particle)
                for particle in self._particles
            ]
        )

        # particle_idx = np.random.multinomial(self._particles.shape[0], similarities, size=self._particle_count)
        # particle_idx = np.random.choice(
        #     self._particles.shape[0],
        #     self._particle_count,
        #     replace=True,
        #     p=similarities / similarities.sum(),
        # )
        # self._particles = self._particles[particle_idx]
        if similarities.sum() == 0:
            similarities += 1.0
        self._particles = np.array(
            random.choices(
                self._particles, weights=similarities, k=self._particle_count
            )
        )

    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        if orientation:
            dx = [math.cos(particle[2]) for particle in self._particles]
            dy = [math.sin(particle[2]) for particle in self._particles]
            axes.quiver(
                self._particles[:, 0],
                self._particles[:, 1],
                dx,
                dy,
                color="b",
                scale=15,
                scale_units="inches",
            )
        else:
            axes.plot(self._particles[:, 0], self._particles[:, 1], "bo", markersize=1)

        return axes

    def show(
        self,
        title: str = "",
        orientation: bool = True,
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            orientation: Draw particle orientation.
            display: True to open a window to visualize the particle filter evolution in real-time.
                Time consuming. Does not work inside a container unless the screen is forwarded.
            block: True to stop program execution until the figure window is closed.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, orientation)

        axes.set_title(title + " (Iteration #" + str(self._iteration) + ")")
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait 1 ms or the figure won't be displayed

        if save_figure:
            save_path = os.path.realpath(
                os.path.join(os.path.dirname(__file__), "..", save_dir, self._timestamp)
            )

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = str(self._iteration).zfill(4) + " " + title.lower() + ".png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _init_particles(
        self,
        particle_count: int,
        global_localization: bool,
        initial_pose: tuple[float, float, float],
        initial_pose_sigma: tuple[float, float, float],
    ) -> np.ndarray:
        """Draws N random valid particles.

        The particles are guaranteed to be inside the map and
        can only have the following orientations [0, pi/2, pi, 3*pi/2].

        Args:
            particle_count: Number of particles.
            global_localization: First localization if True, pose tracking otherwise.
            initial_pose: Approximate initial robot pose (x, y, theta) for tracking [m, m, rad].
            initial_pose_sigma: Standard deviation of the initial pose guess [m, m, rad].

        Returns: A NumPy array of tuples (x, y, theta) [m, m, rad].

        """
        particles = np.empty((particle_count, 3), dtype=object)

        # TODO: 3.4. Complete the missing function body with your code.
        if global_localization:
            x_min, y_min, x_max, y_max = self._map.bounds()
            not_contained = np.ones(particle_count, dtype=np.bool_)
            while any(not_contained):
                particle_to_create_num = np.sum(not_contained)
                particles[not_contained, 0] = (
                    np.random.sample(particle_to_create_num) * (x_max - x_min) + x_min
                )
                particles[not_contained, 1] = (
                    np.random.sample(particle_to_create_num) * (y_max - y_min) + y_min
                )
                particles[not_contained, 2] = np.random.choice(
                    [0, np.pi / 2, np.pi, 3 * np.pi / 2], particle_to_create_num
                )
                not_contained = np.array(
                    list(map(lambda p: not self._map.contains(p[:2]), particles))
                )
        else:
            not_contained = np.ones(particle_count, dtype=np.bool_)
            while any(not_contained):
                particle_to_create_num = np.sum(not_contained)
                particles[not_contained, 0] = np.random.normal(
                    initial_pose[0], initial_pose_sigma[0], particle_to_create_num
                )
                particles[not_contained, 1] = np.random.normal(
                    initial_pose[1], initial_pose_sigma[1], particle_to_create_num
                )
                particles[not_contained, 2] = np.random.normal(
                    initial_pose[2], initial_pose_sigma[2], particle_to_create_num
                )
                not_contained = np.array(
                    list(map(lambda p: not self._map.contains(p[:2]), particles))
                )

        return particles

    def _sense(self, particle: tuple[float, float, float]) -> list[float]:
        """Obtains the predicted measurement of every LiDAR ray given the robot's pose.

        Args:
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns: List of predicted measurements; nan if a sensor is out of range.

        """
        z_hat: list[float] = []

        # TODO: 3.6. Complete the missing function body with your code.
        rays_step = 240 // self._num_rays
        ray_indexes = [r * rays_step for r in range(self._num_rays)]
        for ray in self._lidar_rays(particle, ray_indexes):
            intersection, distance = self._map.check_collision(ray, True)
            if intersection:
                z_hat.append(distance)
            else:
                z_hat.append(self._sensor_range_max)
        z_hat = [
            z if z >= self._sensor_range_min and z <= self._sensor_range_max else np.nan
            for z in z_hat
        ]

        return z_hat

    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean.
            sigma: Standard deviation.
            x: Variable.

        Returns:
            float: Gaussian value.

        """
        # TODO: 3.7. Complete the function body (i.e., replace the code below).
        return np.exp(-0.5 * ((x - mu) / sigma) ** 2) / (sigma * np.sqrt(2 * np.pi))
        diff = x - mu
        return np.exp(-0.5 * np.sum(np.square(diff)) / sigma**2)
        return np.exp(
            -0.5 * (x - mu) @ (x - mu).T / sigma**2
        )  # / (sigma * np.sqrt(2 * np.pi))

    def _lidar_rays(
        self,
        pose: tuple[float, float, float],
        indices: tuple[float],
        degree_increment: float = 1.5,
    ) -> list[list[tuple[float, float]]]:
        """Determines the simulated LiDAR ray segments for a given robot pose.

        Args:
            pose: Robot pose (x, y, theta) in [m] and [rad].
            indices: Rays of interest in counterclockwise order (0 for to the forward-facing ray).
            degree_increment: Angle difference of the sensor between contiguous rays [degrees].

        Returns: Ray segments. Format:
                 [[(x0_start, y0_start), (x0_end, y0_end)],
                  [(x1_start, y1_start), (x1_end, y1_end)],
                  ...]

        """
        x, y, theta = pose

        # Convert the sensor origin to world coordinates
        x_start = x - 0.035 * math.cos(theta)
        y_start = y - 0.035 * math.sin(theta)

        rays = []

        for index in indices:
            ray_angle = math.radians(degree_increment * index)
            x_end = x_start + self._sensor_range_max * math.cos(theta + ray_angle)
            y_end = y_start + self._sensor_range_max * math.sin(theta + ray_angle)
            rays.append([(x_start, y_start), (x_end, y_end)])

        return rays

    def _measurement_probability(
        self, measurements: list[float], particle: tuple[float, float, float]
    ) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with
        the minimum sensor range to perform the computation because the environment is smaller
        than the maximum range.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) [m, m, rad].

        Returns:
            float: Probability.

        """
        probability = 1.0

        # TODO: 3.8. Complete the missing function body with your code.
        z_hat = self._sense(particle)

        rays_step = 240 // self._num_rays
        measurements = [measurements[r * rays_step] for r in range(self._num_rays)]
        for z, z_hat_i in zip(measurements, z_hat):
            if np.isnan(z_hat_i):
                z_hat_i = self._sensor_range_min  # Maybe something maller

            probability *= self._gaussian(z_hat_i, self._sigma_z, z)
        return probability
        particle_measurements = np.array(
            [
                m if not np.isnan(m) else self._sensor_range_min
                for m in self._sense(particle)
            ]
        )
        # rclpy.logging.get_logger("ma").warn(f"{len(particle_measurements)}, {len(measurements)}")
        num_rays = 16
        rays_step = 240 // num_rays

        particle_measurements = self._sense(particle)

        # measurements = [measurements[r*rays_step] for r in range(num_rays)]
        # probability = self._gaussian(
        #     np.array([m if not np.isnan(m) else self._sensor_range_min for m in measurements]),
        #     self._sigma_z,
        #     particle_measurements,
        # )

        real_measurements = [measurements[r * rays_step] for r in range(num_rays)]

        # Calcular la probabilidad para cada par de mediciones
        for z_real, z_pred in zip(real_measurements, particle_measurements):
            # Manejar valores NaN
            if np.isnan(z_pred):
                z_pred = self._sensor_range_min
            if np.isnan(z_real):
                z_real = self._sensor_range_min

            # Calcular la probabilidad con una distribución gaussiana clásica
            prob = np.exp(-0.5 * ((z_real - z_pred) / self._sigma_z) ** 2)
            probability *= prob

        return probability
