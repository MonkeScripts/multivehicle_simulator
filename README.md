# Multivehicle Simulator
This package provides a simulation environment for multiple vehicles (PX4, Ardusub, USV) using ROS 2 and Gazebo Harmonic. It includes launch files, configuration files, and models to facilitate the simulation of various types of vehicles.

## Setting up the environment
The simulation uses rocker to setup a dockerized environment with all the necessary dependencies. To get started, follow these steps:
1. Install Docker
2. Install vcstool:
    ```bash
    sudo apt-get update
    sudo apt-get install python3-vcstool
    ```
3. In your colcon workspace, clone this repository:
    ```bash
    mkdir -p ~/colcon_ws/src
    cd ~/colcon_ws/src
    git clone https://github.com/MonkeScripts/multivehicle_simulator.git
    ```
4. Import the required repositories into `src/` (as siblings of this package), including submodules:
    ```bash
    cd ~/colcon_ws/src
    vcs import --recursive < multivehicle_simulator/multivehicle_simulator.repos
    ```
5. Install [rocker](https://github.com/osrf/rocker) — it wraps `docker run` with GPU and X11
   support. Install it into a dedicated virtual environment:
    ```bash
    sudo apt-get install python3-venv
    python3 -m venv ~/rocker_venv
    . ~/rocker_venv/bin/activate
    pip install git+https://github.com/osrf/rocker.git
    ```
    > Re-activate the venv in any new terminal before building/running:
    > `. ~/rocker_venv/bin/activate`.
6. Build the image (`build.bash` defaults to the `mvsim/` Dockerfile dir and the `humble` tag):
    ```bash
    ./build.bash
    # Tags as mvsim:humble and mvsim:YYYY_MM_DD_HHMM
    ```
7. Run the container:
    ```bash
    ./run.bash mvsim:humble
    ```
    The px4 build would be found in `/root/px4/` inside the container.
    The ardusub build would be found in `/root/auv/` inside the container.
    All files would be mounted to /home/HOST/ inside the container. We would be running the simulation from `/home/HOST/<colcon_ws>/` inside the container.

> The Docker image is self-contained: all dependencies, the PX4 SITL build, the
> ArduSub + ardupilot_gazebo build, and the `bb_robotx_dashboard` toolchain
> (Node 20, a pinned protoc 33, and the FastAPI pip deps) are baked in by
> `mvsim/Dockerfile` — there are no build-time helper scripts to mount.

### Setting up the dashboard
The `bb_robotx_dashboard` source is not copied into the image; it lives in the
mounted workspace. Both the dashboard and its shared ROS interface package
`bb_robotx_msgs` (`BeaconState`, `IncidentZoneReport`, `SpawnIncident` /
`ClearIncident`, `DockLedChoice`, `PipeLedChoice`) are pulled by
`vcs import` from the manifest, so the `--packages-up-to bb_robotx_dashboard`
build below compiles the messages first. On the first shell in the container,
`mvsim/scripts/setup_dashboard.sh` (hooked from `~/.bashrc`) automatically
clones the `robocommand` proto source and generates the Python proto bindings.
The frontend and the colcon build are left to you:
```bash
# inside the container, once
cd /home/HOST/dave_ws/src/bb_robotx_dashboard/frontend && npm ci && npm run build
cd ~/dave_ws && colcon build --packages-up-to bb_robotx_dashboard && source install/setup.bash

# laptop side — web UI on http://localhost:8080
ros2 launch bb_robotx_dashboard dashboard.launch.py
# sim side — LED driver + incident manager
ros2 launch bb_robotx_dashboard robotx_2026_sim.launch.py
```
The dashboard runtime env vars (`ROBOCOMMAND_*`, `DASHBOARD_*`) are baked into
`~/.bashrc`; rotate `DASHBOARD_ADMIN_SECRET` in `mvsim/Dockerfile` and rebuild
before exposing the dashboard beyond localhost.

## Launching the simulation
### Launching the bluerov simulation
The world and the vehicle are launched separately so a vehicle can attach to an already-running
world. Start the world in one terminal:
```bash
ros2 launch multivehicle_sim world.launch.py world_name:=robotx_2026_sg_river
```
`world.launch.py` starts Gazebo with the RobotX 2026 Singapore River course (`robotx_2026_sg_river`
is the default) and publishes the world-level static TFs (`world→map`, `world→world_ned`,
`map→map_ned`).

Then, in a second terminal, attach the BlueROV2 to the running world:
```bash
ros2 launch multivehicle_sim bluerov.launch.py
```
`bluerov.launch.py` spawns the BlueROV2 via `ros_gz_sim create` and brings up ArduSub, MAVROS, and
the ROS↔gz bridge. Pass `world_name:=<world>` to match the world you launched (it is read only to
derive the ArduSub home lat/lon) and `x:=… y:=… yaw:=…` to set the spawn pose. You can open QGC to
connect to the vehicle using the appropriate UDP port (14550).

### Running bluerov mission
>Note: Ensure that QGC is running and connected to the Ardusub vehicle before executing the mission demo.

A simple state machine mission is provided to demonstrate autonomous movement of the BlueROV2 in the simulation. To run the mission, use the following command:
```bash
ros2 launch multivehicle_sim bluerov_mission.launch.py
```
This will command the BlueROV2 to dive to a depth of 2 meters and move in a square pattern. During each leg of the square, the vehicle would also yaw to head in the direction of the next waypoint.

### Launching the boat (BlueBoat USV) simulation
Same world-first flow as the BlueROV2. With a world already running
(`world.launch.py`), spawn the BlueBoat USV in a second terminal:
```bash
ros2 launch multivehicle_sim boat.launch.py
```
`boat.launch.py` attaches the BlueBoat to the running world via `ros_gz_sim create`
and brings up the ROS↔gz bridge — it does not start its own Gazebo. The spawn pose
defaults to `x:=10.0 y:=-390.0`; override with `x:=… y:=… yaw:=…`, and set
`namespace:=…` (default `blueboat`) to run more than one.

Unlike the BlueROV2 (ArduSub/MAVROS), the boat runs a custom control stack rather
than an autopilot. Start it with:
```bash
ros2 launch multivehicle_sim boat_control.launch.py
```
This brings up the thrust mixer (`cmd_vel` → left/right thrust) and the LOS
`blueboat_waypoint_controller`. Useful args: `spawn:=true` also spawns the boat in
the same launch, `use_mission:=true` runs the demo square course, and
`use_controller:=false` hands `/blueboat/cmd_vel` to nav2 instead of the LOS controller.

### Adding the PX4 drone
> Note: The setup and scripts are adapted from https://github.com/Dronecode/roscon-25-workshop/tree/main. For more comprehensive applications of drone simulation with ROS 2 and Gazebo, please refer to the original repository.
To add a PX4 drone to the simulation, run the following command in a new terminal inside the rocker container:
```bash
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4010 PX4_UXRCE_DDS_NS=x500 PX4_PARAM_UXRCE_DDS_SYNCT=0 PX4_GZ_MODEL_POSE="47.40,-388.95,3.85,0,0.0" /root/px4/px4_sitl/bin/px4 -w /root/px4/px4_sitl/romfs -i 1
```
The individual CLI arguments for the PX4 command are:
- `PX4_GZ_STANDALONE=1`: Runs PX4 in standalone mode with Gazebo, attaches to an existing Gazebo instance
- `PX4_SYS_AUTOSTART=4010`: Sets the airframe type (4010 is `gz_x500_mono_cam`, an x500 with a front mono camera; 4001 is the plain quadrotor, in which case pass `camera:=false model_name:=x500_1` to `uav_gz.launch.py`)
- `PX4_UXRCE_DDS_NS=x500`: Namespaces the drone's uXRCE-DDS (ROS 2) topics as `/x500/fmu/...` (overrides PX4's default `px4_<instance>`). Must match `vehicle_namespace` in `px4_offboard.launch.py`.
- `PX4_PARAM_UXRCE_DDS_SYNCT=0`: Disables DDS sync timing
- `PX4_GZ_MODEL_POSE`: Sets the initial pose; fields are x, y, z, roll, pitch, yaw (trailing fields optional)
- `-w`: Specifies the working directory path for PX4 SITL
- `-i 1`: Sets the instance ID to 1 (must be unique per vehicle). Note that the flag specifies the instance ID. The ardusub instance uses ID 0, so make sure to use a different ID for the PX4 instance to avoid conflicts.

This will start a PX4 SITL instance that connects to the Gazebo simulation. You can then connect to the PX4 vehicle using QGC on UDP port 14550 (shared with ArduSub — use QGC's vehicle selector, see below). 

You should now see the default PX4 airframe in the Gazebo simulation environment. You can also open QGC to connect to the PX4 vehicle using the appropriate UDP port (14550). If ardusub is using 14550, you can **toggle the different vehicles in QGC using the vehicle selector at the top left corner of the interface.**

### Running offboard demo for PX4
>Note: Ensure that QGC is running and connected to the PX4 vehicle before executing the offboard demo.
1. In one terminal inside the rocker container, run the Microxrce-DDS agent to facilitate communication between ROS 2 and PX4:
    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```
2. In another terminal inside the rocker container, source your ROS 2 workspace and run the offboard demo launch file:
    ```bash
        ros2 launch multivehicle_sim px4_offboard.launch.py
    ```
    In this launch file, you need to check the following parameters:
    - `vehicle_namespace`: Prepended value to the PX4 topics. Must match `PX4_UXRCE_DDS_NS` set on the PX4 SITL process (e.g., `x500`). Confirm with `ros2 topic list` or the MicroXRCEAgent logs.
    - `vehicle_id`: The MAVLink system id (`target_system`), which is the PX4 instance + 1. So if you launched PX4 with `-i 1`, set this to `2`.

This will start the offboard control demo, which commands the PX4 vehicle to take off, hover, and land autonomously.



## Issues and fixes
### [ros_gz_sim]: Requesting list of world names.
When using the `ros_gz_sim` package, you may encounter an issue when requesting the list of world names. This happens because gazebo is trying to render the world before it has fully loaded all the necessary assets. 

This most likely happens because gazebo is downloading the Singapore river model from Gazebo Fuel, an online database of models and worlds.  Depending on your internet connection and the size of the high-res textures, this can make the initial load feel like it's hanging.

> Note that we only incur this penalty once. Once a model is downloaded, it is cached in your local folder (usually ~/.gz/fuel or ~/.ignition/fuel). Subsequent loads should be near-instant.

## License

First-party code and models in this repository are licensed under the
**Apache License 2.0** — see [`LICENSE`](LICENSE).

This repository also bundles and references third-party 3D models, meshes, and
textures (e.g. the BlueROV2 and BlueBoat models, the Pier, and Gazebo Fuel
assets) that remain under their **own** licenses and are **not** covered by the
Apache-2.0 grant. See [`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md) for
the full inventory, origins, licenses, and attributions.

