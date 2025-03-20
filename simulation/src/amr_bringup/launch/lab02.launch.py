from launch import LaunchDescription

# import socket
from launch_ros.actions import LifecycleNode, Node

import math


def generate_launch_description():
    start = (1.0, -1.0, 0.5 * math.pi)  # Outer corridor
    # start = (0.6, -0.6, 1.5 * math.pi)  # Inner corridor

    wall_follower_node = LifecycleNode(
        package="amr_control",
        executable="wall_follower",
        name="wall_follower",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
    )

    coppeliasim_node = LifecycleNode(
        package="amr_simulation",
        executable="coppeliasim",
        name="coppeliasim",
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {"start": start, "ip": "172.18.32.1"}
        ],  # ip = "" if you are going to use docker
    )

    lifecycle_manager_node = Node(
        package="amr_bringup",
        executable="lifecycle_manager",
        output="screen",
        arguments=["--ros-args", "--log-level", "WARN"],
        parameters=[
            {
                "node_startup_order": (
                    "wall_follower",
                    "coppeliasim",  # Must be started last
                )
            }
        ],
    )

    return LaunchDescription(
        [
            wall_follower_node,
            coppeliasim_node,
            lifecycle_manager_node,  # Must be launched last
        ]
    )


# def read_ip() -> str :
#     st = socket.socket(socket.AF_INET , socket.SOCK_DGRAM )

#     try:
#         st.connect(("10.255.255.255", 1)) # No importa la dirección
#         ip = st.getsockname()[0]

#     except Exception : # Si está en local
#         ip = "127.0.0.1"

#     finally :
#         st.close() # Cerramos el socket

#     return ip
