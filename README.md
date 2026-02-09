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
3. Navigate to the multivehicle_simulator directory and import the required repositories using vcs:
    ```bash
    cd multivehicle_simulator
    vcs import < multivehicle_simulator.repos
    ```
3. Setup a python virtual environment and install rocker:
    ```bash
    python3 -m venv multivehicle_venv
    source multivehicle_venv/bin/activate
    ```
4. Install [rocker](https://github.com/osrf/rocker) inside the virtual environment. It should already be cloned by importing the repos in the previous step. If not, you can clone it manually and then install it:
    ```bash
    git clone https://github.com/osrf/rocker.git
    python3 -m pip install rocker/.
    ```
5. Build the rocker image with the required dependencies:
    ```bash
    ./build.bash humble_hybrid
    ```
6. To run the rocker container, run:
    ```bash
     ./run.bash dockwater:humble_hybrid
    ```
    The px4 build would be found in `/root/px4/` inside the container.
    The ardusub build would be found in `/root/auv/` inside the container.
    All files would be mounted to /home/HOST/ inside the container. We would be running the simulation from `/home/HOST/<colcon_ws>/` inside the container.

## Launching the simulation
### Launching the bluerov simulation
Once inside the rocker container, you can launch the multivehicle simulation using the provided launch files. For example, to launch a simulation with a BlueROV2 vehicle, run:
```bash
ros2 launch multivehicle_simulator bluerov.launch.py world_name:=nbpark
```
This will start Gazebo with the BlueROV2 model in Nathan Benderson park. You can open QGC to connect to the vehicle using the appropriate UDP port (14550).

### Running bluerov mission
>Note: Ensure that QGC is running and connected to the Ardusub vehicle before executing the mission demo.
A simple state machine mission is provided to demonstrate autonomous movement of the BlueROV2 in the simulation. To run the mission, use the following command:
```bash
ros2 launch multivehicle_simulator bluerov_mission.launch.py
```
This will command the BlueROV2 to dive to a depth of 2 meters and move in a square pattern. During each leg of the square, the vehicle would also yaw to head in the direction of the next waypoint.

### Adding the PX4 drone
> Note: The setup and scripts are adapted from https://github.com/Dronecode/roscon-25-workshop/tree/main. For more comprehensive applications of drone simulation with ROS 2 and Gazebo, please refer to the original repository.
To add a PX4 drone to the simulation, run the following command in a new terminal inside the rocker container:
```bash
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_PARAM_UXRCE_DDS_SYNCT=0 PX4_GZ_MODEL_POSE="-108.860000,603.150000,0.8,0,0.0" /root/px4/px4_sitl/bin/px4 -w /root/px4/px4_sitl/romfs -i 1
```
The individual CLI arguments for the PX4 command are:
- `PX4_GZ_STANDALONE=1`: Runs PX4 in standalone mode with Gazebo, attaches to an existing Gazebo instance
- `PX4_SYS_AUTOSTART=4001`: Sets the airframe type (4001 is the generic quadrotor)
- `PX4_PARAM_UXRCE_DDS_SYNCT=0`: Disables DDS sync timing
- `PX4_GZ_MODEL_POSE`: Sets the initial position and orientation (x, y, z, roll, pitch, yaw)
- `-w`: Specifies the working directory path for PX4 SITL
- `-i 1`: Sets the instance ID to 1 (must be unique per vehicle). Note that the flag specifies the instance ID. The ardusub instance uses ID 0, so make sure to use a different ID for the PX4 instance to avoid conflicts.

This will start a PX4 SITL instance that connects to the Gazebo simulation. You can then connect to the PX4 vehicle using QGC on UDP port 14540. 

You should now see the default PX4 airframe in the Gazebo simulation environment. You can also open QGC to connect to the PX4 vehicle using the appropriate UDP port (14550). If ardusub is using 14550, you can **toggle the different vehicles in QGC using the vehicle selector at the top left corner of the interface.**

### Running offboard demo for PX4
>Note: Ensure that QGC is running and connected to the PX4 vehicle before executing the offboard demo.
1. In one terminal inside the rocker container, run the Microxrce-DDS agent to facilitate communication between ROS 2 and PX4:
    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```
2. In another terminal inside the rocker container, source your ROS 2 workspace and run the offboard demo launch file:
    ```bash
        ros2 launch multivehicle_simulator px4_offboard.launch.py
    ```
    In this launch file, you need to check the following parameters:
    - `vehicle_namespace`: Prepended value to the PX4 topics. Ensure it matches the namespace used by the PX4 vehicle in the simulation (e.g., `px4_1`). Please check the topics being published by PX4 to confirm the correct namespace through`ros2 topic list` or viewing the logs on MicroXRCEAgent.
    - `vehicle_id`: The instance ID of the PX4 vehicle (should match the ID used when launching PX4, e.g., `1`). This index is indexed from 1, so if you used `-i 1` when launching PX4, set this parameter to `2`.

This will start the offboard control demo, which commands the PX4 vehicle to take off, hover, and land autonomously.



## Issues and fixes
### MESA: error: ZINK: vkCreateInstance failed (VK_ERROR_INCOMPATIBLE_DRIVER)
>Note that this issue is tested with the outdated fork of rocker from IOES LAB. If you are using the official rocker repository, the issue might not be present.

When running the simulation inside the rocker container, you might encounter the following error related to MESA and ZINK:
```MESA: error: ZINK: vkCreateInstance failed (VK_ERROR_INCOMPATIBLE_DRIVER)```
This error indicates that the Vulkan driver is not compatible with the ZINK driver being used by MESA. This can happen if the host system's graphics drivers do not support Vulkan or if there is a mismatch between the host and container graphics configurations. As such I changed the Nvidia options in rocker to use `--gpus all` on top of `--runtime=nvidia`.
#### Fix
Edit the get_docker_args function in `rocker/src/rocker/nvidia_extension.py` to the following:
```python
def get_docker_args(self, cliargs):
    force_flag = cliargs.get('nvidia', None)
    print(f"FORCE_FLAG: {force_flag}")

    # MODIFIED: Added --gpus all to ensure visibility
    if force_flag == 'runtime':
        return "  --runtime=nvidia" 
    if force_flag == 'gpus':
        return "  --gpus all --runtime=nvidia"
    if get_docker_version() >= Version("19.03"):
        return "  --gpus all --runtime=nvidia"
    
    return " --gpus all --runtime=nvidia"
```
The simulation should now run without the MESA ZINK error.

Rebuild the package to apply the changes. Run the following command from the root of the repository to install the package in editable mode:

```bash
pip install -e .
```

### [ros_gz_sim]: Requesting list of world names.
When using the `ros_gz_sim` package, you may encounter an issue when requesting the list of world names. This happens because gazebo is trying to render the world before it has fully loaded all the necessary assets. 

One possibility would be that gazebo is downloading some models from Gazebo Fuel, an online database of models and worlds. When you launch a world that references an asset you don't have locally, Gazebo tries to download it on the fly. Depending on your internet connection and the size of the high-res textures, this can make the initial load feel like it's hanging.

> Note that we only incur this penalty once. Once a model is downloaded, it is cached in your local folder (usually ~/.gz/fuel or ~/.ignition/fuel). Subsequent loads should be near-instant.

Depending on the world, sometimes the models do not have any collision geometry enabled, which makes it difficult to add vehicles without them falling through the ground.

#### Possible fix
To fix this issue, you can pre-download the necessary models and worlds from Gazebo Fuel before launching your simulation. This way, Gazebo won't need to download them during runtime, which should resolve the hanging issue.
The Nathan Benderson Park is already downloaded in the `multivehicle_simulator` package under `worlds/nathan_benderson_park`. However, if you want to download other models or worlds, you can follow the steps for the Nathan Benderson Park as an example:

Example with Nathan Benderson Park world:
1. Download the zipped model from this [website](https://app.gazebosim.org/OpenRobotics/fuel/models/nathan_benderson_park). You might need to create a free account to access the download.
2. Unzip the downloaded file.
3. Move the unzipped folder to the `models` directory.
4. Replace the world file path in your launch file to point to the local copy of the world file.
```xml
    <include>
      <pose>0 0 -1 0 0 0</pose>
      <uri>model://nathan_benderson_park</uri>
    </include>


    <!-- Old configuration where we have not downloaded the nathan_benderson_park model -->
    <!-- <include>
      <pose>0 0 -1 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/nathan_benderson_park</uri>
    </include> -->
```
5. Build and launch your simulation again. Gazebo should now load the world without hanging.
> One benefit of this approach is that you can also ****modify the models locally if needed, such as enabling collision geometry or adjusting textures**.
