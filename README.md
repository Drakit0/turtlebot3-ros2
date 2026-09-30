# turtlebot3-ros2

ROS 2 workspaces for the TurtleBot3 Burger, written for the course Robots Móviles y Autónomos (Universidad Pontificia Comillas, ETSI ICAI), academic year 2024-2025. The repository has two parts that share most of their packages: a simulation in CoppeliaSim and the version adapted to run on the real robot.

Authors: Pablo Tuñón Laguna, Lydia Ruiz Martínez and Jorge Vančo Sampedro.

## What is implemented

Both workspaces contain `amr_bringup`, `amr_control`, `amr_localization`, `amr_msgs` and `amr_planning`. Each node being a lifecycle node started in order by a lifecycle manager.

- `amr_localization`: a particle filter (`particle_filter.py`, with DBSCAN clustering of the particles for global localization) and, in the simulation workspace only, an extended Kalman filter (`ekf.py`) with predict and update steps from laser scans. The filters use the `Map` class (`maps.py`), which loads a JSON map and casts rays against the walls. `maps.hpp` and `maps.cpp` are a C++ version of that map class written with Boost.Geometry and nlohmann/json. They are not part of the colcon build and `maps.cpp` includes `map.hpp`, which does not exist under that name, so the C++ version has not been compiled from this repository.
- `amr_control`: a wall follower state machine (`wall_follower.py`) and a pure pursuit path tracker (`pure_pursuit.py`).
- `amr_planning`: path planning with a probabilistic roadmap (`prm.py`), optionally on a grid, with path smoothing.
- `amr_bringup`: launch files for lab 2 (wall following), lab 3 (particle filter), lab 4 (planning and path tracking) and the final project, plus the lifecycle manager.
- `amr_msgs`: a pose message with a timestamp (`PoseStamped`) and, in `real_robot/`, a `Move` message.

Only in `simulation/`:

- `amr_simulation`: the node that drives CoppeliaSim through its ZMQ remote API and a TurtleBot3 Burger model, with the worlds `lab02`, `lab03` and `project`.
- `Informe_1` and `Informe_2`: the lab reports (LaTeX source and PDF, in Spanish). The `.tex` files include images from a `fonts/` folder (rqt_graph captures and the ICAI logo) that is not in the repository, so they do not compile here; the PDFs are included.

Only in `real_robot/`:

- `amr_turtlebot3`: an odometry node that republishes `/odom` as `/odometry`, and a monitoring node that prints where the robot localized and whether it reached the goal.
- `amr_teleoperation`: a keyboard publisher and a teleoperation node. Both import `KeyboardMsg` from `amr_msgs`, but its definition is not in the repository (`amr_msgs/msg/` holds only `Move.msg` and `PoseStamped.msg`), so these two nodes do not build as they stand.
- `start_robot.sh` and `run.sh`.

## Structure

```
simulation/
  Informe_1.tex, Informe_2.tex (+ PDF)
  src/
    amr_bringup  amr_control  amr_localization  amr_msgs
    amr_planning  amr_simulation
real_robot/
  start_robot.sh, run.sh
  src/
    amr_bringup  amr_control  amr_localization  amr_msgs
    amr_planning  amr_teleoperation  amr_turtlebot3
```

## Build

Requirements: ROS 2 with colcon, and the Python packages imported by the code (numpy, matplotlib, shapely, scikit-learn, pytz, transforms3d). The simulation also needs `coppeliasim_zmqremoteapi_client` and CoppeliaSim.

The intersection routine is loaded from a compiled library. The repository ships `libintersect.dll` and `libintersect.dylib`; on Linux build it from the C source in `amr_localization/amr_localization`:

```
gcc -shared -o libintersect.so -fPIC intersect.c -Wall -g -O3
```

and put a copy of `libintersect.so` next to `maps.py` in `amr_planning/amr_planning` as well, since that package loads the library from its own folder.

Then, from either `simulation/` or `real_robot/`:

```
colcon build
source install/setup.bash
```

## Run the simulation

Open the world from `simulation/src/amr_simulation/worlds/` (`lab02.ttt`, `lab03.ttt` or `project.ttt`) in CoppeliaSim with the ZMQ remote API on port 23000. The simulation node connects to `host.docker.internal`, which matches running ROS 2 in a container with CoppeliaSim on the host. Then launch the matching file:

```
cd simulation
ros2 launch amr_bringup lab02.launch.py          # wall following
ros2 launch amr_bringup lab03.launch.py          # particle filter
ros2 launch amr_bringup lab04.launch.py          # planning and path tracking
ros2 launch amr_bringup project_sim.launch.py    # localization, planning and control together
```

The start pose, goal and filter parameters (2000 particles in the project launch) are set inside the launch files.

## Run on the real robot

On the robot:

```
cd real_robot
./start_robot.sh    # ros2 launch turtlebot3_bringup robot.launch.py
```

On the computer, after building:

```
./run.sh            # source install/setup.sh; ros2 launch amr_bringup project.launch.py
```

`project.launch.py` starts the particle filter, the probabilistic roadmap, the odometry node, the wall follower, the pure pursuit controller and the lifecycle manager. The lab launch files in the same folder start the subsets used in each lab session.

## Results

The reports in `simulation/` answer the lab questionnaires (rqt_graph diagrams and the changes needed to move from simulation to the real robot). They do not state numeric results, and none are claimed here.

## Licence

The authors' own code and reports are under the MIT licence (see `LICENSE`).

Course-provided material is not covered by that licence. It comes from the course scaffolding, whose package metadata names Jaime Boal as maintainer and Apache License 2.0 (a copy is in `real_robot/src/amr_msgs/LICENSE`):

- `intersect.c`, `intersect.h`, `intersect.py`, `libintersect.dll` and `libintersect.dylib` in `amr_localization` and `amr_planning`
- the lifecycle manager in `amr_bringup`
- the CoppeliaSim interface and robot classes in `amr_simulation`, and the worlds and robot model in `simulation/src/amr_simulation/worlds/`
- the map files in `maps/` of `amr_localization` and `amr_planning`
- the package skeletons (`package.xml`, `setup.py`, `setup.cfg`, `test/`) in every package
