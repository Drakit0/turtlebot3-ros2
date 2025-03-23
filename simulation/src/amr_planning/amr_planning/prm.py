import datetime
import numpy as np
import os
import pytz
import random
import time

# This try-except enables local debugging of the PRM class
try:
    from amr_planning.maps import Map
except ImportError:
    from maps import Map

from matplotlib import pyplot as plt


class PRM:
    """Class to plan a path to a given destination using probabilistic roadmaps (PRM)."""

    def __init__(
        self,
        map_path: str,
        obstacle_safety_distance=0.08,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
        sensor_range_max: float = 8.0,
    ):
        """Probabilistic roadmap (PRM) class initializer.

        Args:
            map_path: Path to the map of the environment.
            obstacle_safety_distance: Distance to grow the obstacles by [m].
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].
            sensor_range_max: Sensor measurement range [m].
        """
        self._map: Map = Map(
            map_path,
            sensor_range=sensor_range_max,
            safety_distance=obstacle_safety_distance,
            compiled_intersect=False,
            use_regions=False,
        )

        self._graph: dict[tuple[float, float], list[tuple[float, float]]] = (
            self._create_graph(
                use_grid,
                node_count,
                grid_size,
                connection_distance,
            )
        )

        self._figure, self._axes = plt.subplots(1, 1, figsize=(7, 7))
        self._timestamp = datetime.datetime.now(
            pytz.timezone("Europe/Madrid")
        ).strftime("%Y-%m-%d_%H-%M-%S")

    def find_path(
        self, start: tuple[float, float], goal: tuple[float, float]
    ) -> list[tuple[float, float]]:
        """Computes the shortest path from a start to a goal location using the A* algorithm.

        Args:
            start: Initial location in (x, y) [m] format.
            goal: Destination in (x, y) [m] format.

        Returns:
            Path to the destination. The first value corresponds to the initial location.

        """
        # Check if the target points are valid
        if not self._map.contains(start):
            raise ValueError("Start location is outside the environment.")

        if not self._map.contains(goal):
            raise ValueError("Goal location is outside the environment.")

        ancestors: dict[tuple[float, float], tuple[float, float]] = (
            {}
        )  # {(x, y: (x_prev, y_prev)}

        # TODO: 4.3. Complete the function body (i.e., replace the code below).
        path: list[tuple[float, float]] = []

        closest_init = None
        closest_init_dist = np.inf
        closest_final = None
        closest_final_dist = np.inf

        for point in self._graph.keys():
            init_dist = np.linalg.norm(np.array(point) - np.array(start))
            final_dist = np.linalg.norm(np.array(point) - np.array(goal))

            if init_dist < closest_init_dist:
                closest_init_dist = init_dist
                closest_init = point

            if final_dist < closest_final_dist:
                closest_final_dist = final_dist
                closest_final = point

        open_list = {
            closest_init: (
                np.linalg.norm(np.array(closest_init) - np.array(closest_final)),
                0,
            )
        }

        if closest_init != start:
            ancestors[closest_init] = start

        if closest_final != goal:
            ancestors[closest_final] = goal

        closed_list = set()

        while len(open_list) != 0:

            node = min(open_list, key=lambda k: open_list.get(k)[0])

            g = open_list[node][1]
            del open_list[node]

            for connected in self._graph[node]:
                connected_g = g + np.linalg.norm(np.array(connected) - np.array(node))
                connected_f = connected_g + np.linalg.norm(
                    np.array(closest_final) - np.array(connected)
                )

                if connected not in closed_list:
                    if (
                        connected not in open_list.keys()
                        or connected_f < open_list[connected][0]
                    ):
                        open_list[connected] = (connected_f, connected_g)

                        ancestors[connected] = node

            closed_list.add(node)

            if node == closest_final:
                return self._reconstruct_path(start, goal, ancestors)

        raise FileNotFoundError("Path not found.")

    @staticmethod
    def smooth_path(
        path: list[tuple[float, float]],
        data_weight: float = 0.1,
        smooth_weight: float = 0.3,
        additional_smoothing_points: int = 0,
        tolerance: float = 1e-6,
    ) -> list[tuple[float, float]]:
        """Computes a smooth path from a piecewise linear path.

        Args:
            path: Non-smoothed path to the goal (start location first).
            data_weight: The larger, the more similar the output will be to the original path.
            smooth_weight: The larger, the smoother the output path will be.
            additional_smoothing_points: Number of equally spaced intermediate points to add
                between two nodes of the original path.
            tolerance: The algorithm will stop when after an iteration the smoothed path changes
                less than this value.

        Returns: Smoothed path (initial location first) in (x, y) format.

        """
        smoothed_path = []

        # Add intermediate points if required
        if additional_smoothing_points != 0:
            for point_1, point_2 in zip(path, path[1:]):
                smoothed_path.append(point_1)
                for i in range(additional_smoothing_points):
                    px = point_1[0] + (i + 1) * (point_2[0] - point_1[0]) / (additional_smoothing_points + 1)
                    py = point_1[1] + (i + 1) * (point_2[1] - point_1[1]) / (additional_smoothing_points + 1)
                    smoothed_path.append((px, py))
            smoothed_path.append(path[-1])
        else:
            smoothed_path = path[:]

        # Convert to numpy array for easier manipulation
        smoothed_path = np.array(smoothed_path)
        original_path = np.array(smoothed_path)

        # Iterative smoothing
        change = tolerance
        while change >= tolerance:
            change = 0.0
            
            for i in range(1, len(smoothed_path) - 1):
                old_x, old_y = smoothed_path[i]
                
                smoothed_path[i][0] += data_weight * (original_path[i][0] - smoothed_path[i][0])
                smoothed_path[i][0] += smooth_weight * (smoothed_path[i - 1][0] + smoothed_path[i + 1][0] - 2 * smoothed_path[i][0])
                smoothed_path[i][1] += data_weight * (original_path[i][1] - smoothed_path[i][1])
                smoothed_path[i][1] += smooth_weight * (smoothed_path[i - 1][1] + smoothed_path[i + 1][1] - 2 * smoothed_path[i][1])
                
                change += abs(smoothed_path[i][0] - old_x) + abs(smoothed_path[i][1] - old_y)

        return smoothed_path.tolist()

    def plot(
        self,
        axes,
        path: list[tuple[float, float]] = (),
        smoothed_path: list[tuple[float, float]] = (),
    ):
        """Draws particles.

        Args:
            axes: Figure axes.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).

        Returns:
            axes: Modified axes.

        """
        # Plot the nodes
        x, y = zip(*self._graph.keys())
        axes.plot(list(x), list(y), "co", markersize=1)

        # Plot the edges
        for node, neighbors in self._graph.items():
            x_start, y_start = node

            if neighbors:
                for x_end, y_end in neighbors:
                    axes.plot([x_start, x_end], [y_start, y_end], "c-", linewidth=0.25)

        # Plot the path
        if path:
            x_val = [x[0] for x in path]
            y_val = [x[1] for x in path]

            axes.plot(x_val, y_val)  # Plot the path
            axes.plot(
                x_val[1:-1], y_val[1:-1], "bo", markersize=4
            )  # Draw nodes as blue circles

        # Plot the smoothed path
        if smoothed_path:
            x_val = [x[0] for x in smoothed_path]
            y_val = [x[1] for x in smoothed_path]

            axes.plot(x_val, y_val, "y")  # Plot the path
            axes.plot(
                x_val[1:-1], y_val[1:-1], "yo", markersize=2
            )  # Draw nodes as yellow circles

        if path or smoothed_path:
            axes.plot(
                x_val[0], y_val[0], "rs", markersize=7
            )  # Draw a red square at the start location
            axes.plot(
                x_val[-1], y_val[-1], "g*", markersize=12
            )  # Draw a green star at the goal location

        return axes

    def show(
        self,
        title: str = "",
        path=(),
        smoothed_path=(),
        display: bool = False,
        block: bool = False,
        save_figure: bool = False,
        save_dir: str = "images",
    ):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            path: Path (start location first).
            smoothed_path: Smoothed path (start location first).
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
        axes = self.plot(axes, path, smoothed_path)

        axes.set_title(title)
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=block)
            plt.pause(0.001)  # Wait for 1 ms or the figure won't be displayed

        if display:
            plt.show(block=block)

        if save_figure:
            save_path = os.path.join(os.path.dirname(__file__), "..", save_dir)

            if not os.path.isdir(save_path):
                os.makedirs(save_path)

            file_name = f"{self._timestamp} {title.lower()}.png"
            file_path = os.path.join(save_path, file_name)
            figure.savefig(file_path)

    def _connect_nodes(
        self,
        graph: dict[tuple[float, float], list[tuple[float, float]]],
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Connects every generated node with all the nodes that are closer than a given threshold.

        Args:
            graph: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A modified graph with lists of connected nodes as values.

        """
        # TODO: 4.2. Complete the missing function body with your code.

        keys = list(graph.keys())
        for i, point_1 in enumerate(keys):
            for j, point_2 in enumerate(keys[i + 1 :]):

                dist = np.linalg.norm(np.array(point_1) - np.array(point_2))

                if dist < connection_distance and not self._map.crosses((point_1, point_2)):
                    graph[point_1].append(point_2)
                    graph[point_2].append(point_1)

        return graph

    def _create_graph(
        self,
        use_grid: bool = False,
        node_count: int = 50,
        grid_size=0.1,
        connection_distance: float = 0.15,
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a roadmap as a graph with edges connecting the closest nodes.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.
            connection_distance: Maximum distance to consider adding an edge between two nodes [m].

        Returns: A dictionary with (x, y) [m] tuples as keys and lists of connected nodes as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        graph = self._generate_nodes(use_grid, node_count, grid_size)
        graph = self._connect_nodes(graph, connection_distance)

        return graph

    def _generate_nodes(
        self, use_grid: bool = False, node_count: int = 50, grid_size=0.1
    ) -> dict[tuple[float, float], list[tuple[float, float]]]:
        """Creates a set of valid nodes to build a roadmap with.

        Args:
            use_grid: Sample from a uniform distribution when False.
                Use a fixed step grid layout otherwise.
            node_count: Number of random nodes to generate. Only considered if use_grid is False.
            grid_size: If use_grid is True, distance between consecutive nodes in x and y.

        Returns: A dictionary with (x, y) [m] tuples as keys and empty lists as values.
            Key elements are rounded to a fixed number of decimal places to allow comparisons.

        """
        graph: dict[tuple[float, float], list[tuple[float, float]]] = {}

        # TODO: 4.1. Complete the missing function body with your code.

        x_min, y_min, x_max, y_max = self._map.bounds()
        x_range = x_max - x_min
        y_range = y_max - y_min

        if use_grid:
            for x in range(int(x_range // grid_size) + 1):
                for y in range(int(y_range // grid_size) + 1):
                    point = (np.round(x_min + x * grid_size, 2), np.round(y_min + y * grid_size, 2))

                    if self._map.contains(point):
                        graph[point] = []

        else:
            while len(graph) < node_count:
                point_x = np.round(np.random.uniform(x_min, x_max), 2)
                point_y = np.round(np.random.uniform(y_min, y_max), 2)

                if self._map.contains((point_x, point_y)):
                    graph[point] = []

        return graph

    def _reconstruct_path(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
        ancestors: dict[tuple[int, int], tuple[int, int]],
    ) -> list[tuple[float, float]]:
        """Computes the path from the start to the goal given the ancestors of a search algorithm.

        Args:
            start: Initial location in (x, y) [m] format.
            goal: Goal location in (x, y) [m] format.
            ancestors: Dictionary with (x, y) [m] tuples as keys and the node (x_prev, y_prev) [m]
                from which it was added to the open list as values.

        Returns: Path to the goal (start location first) in (x, y) [m] format.

        """
        path: list[tuple[float, float]] = [goal]

        # TODO: 4.4. Complete the missing function body with your code.

        node = goal
        path.append(node)

        while node != start:
            node = ancestors[node]
            path.append(node)

        return path[::-1]


if __name__ == "__main__":
    map_name = "project"
    map_path = os.path.realpath(
        os.path.join(os.path.dirname(__file__), "..", "maps", map_name + ".json")
    )

    # Create the roadmap
    start_time = time.perf_counter()
    prm = PRM(
        map_path, use_grid=True, node_count=250, grid_size=0.1, connection_distance=0.15
    )
    roadmap_creation_time = time.perf_counter() - start_time

    print(f"Roadmap creation time: {roadmap_creation_time:1.3f} s")

    # Find the path
    start_time = time.perf_counter()
    path = prm.find_path(start=(-1.0, -1.0), goal=(-0.6, 1.0))
    pathfinding_time = time.perf_counter() - start_time

    print(f"Pathfinding time: {pathfinding_time:1.3f} s")

    # Smooth the path
    start_time = time.perf_counter()
    smoothed_path = prm.smooth_path(
        path, data_weight=0.1, smooth_weight=0.3, additional_smoothing_points=3
    )
    smoothing_time = time.perf_counter() - start_time

    print(f"Smoothing time: {smoothing_time:1.3f} s")

    prm.show(path=path, smoothed_path=smoothed_path, save_figure=True)
