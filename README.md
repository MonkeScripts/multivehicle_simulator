# Multivehicle Simulator
This package provides a simulation environment for multiple vehicles (PX4, Ardusub, USV) using ROS 2 and Gazebo Harmonic. It includes launch files, configuration files, and models to facilitate the simulation of various types of vehicles.

# -- Documentation -- WIP, not complete

## Setting up the environment
The simulation uses rocker to setup a dockerized environment with all the necessary dependencies. To get started, follow these steps:
1. Install Docker
2. Setup a python virtual environment and install rocker:
    ```bash
    python3 -m venv multivehicle_venv
    source multivehicle_venv/bin/activate
    ```
3. Install [rocker](https://github.com/osrf/rocker):
    ```bash
    ```bash
    python3 -m pip install rocker/.
    ```
4. Build the rocker image with the required dependencies:
    ```bash
    ./build.bash humble_hybrid
    ```
5. To run the rocker container, run:
    ```bash
     ./run.bash dockwater:humble_hybrid
    ```
## Launching the simulation
### Launching the bluerov simulation
Once inside the rocker container, you can launch the multivehicle simulation using the provided launch files. For example, to launch a simulation with a BlueROV2 vehicle, run:
```bash
ros2 launch multivehicle_simulator bluerov.launch.py
```
This will start Gazebo with the BlueROV2 model and the necessary ROS 2 nodes for simulation. You can open QGC to connect to the vehicle using the appropriate UDP port (14550).
### Adding the PX4 drone
To add a PX4 drone to the simulation, run the following command in a new terminal inside the rocker container:
```bash
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_PARAM_UXRCE_DDS_SYNCT=0 /root/px4/px4_sitl/bin/px4 -w /root/px4/px4_sitl/romfs -i 3
```
This will start a PX4 SITL instance that connects to the Gazebo simulation. You can then connect to the PX4 vehicle using QGC on UDP port 14540. 
> Note that the `-i` flag specifies the instance ID. The ardusub instance uses ID 1, so make sure to use a different ID for the PX4 instance to avoid conflicts.

You should now see the default PX4 airframe in the Gazebo simulation environment. You can also open QGC to connect to the PX4 vehicle using the appropriate UDP port (14550). If ardusub is using 14550, you can toggle the different vehicles in QGC using the vehicle selector at the top left corner of the interface.

## Issues and fixes
### MESA: error: ZINK: vkCreateInstance failed (VK_ERROR_INCOMPATIBLE_DRIVER)
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

### [ros_gz_sim]: Requesting list of world names.
When using the `ros_gz_sim` package, you may encounter an issue when requesting the list of world names. This happens because gazebo is trying to download some models from Gazebo Fuel.

Gazebo Fuel is an online database of models and worlds. When you launch a world that references an asset you don't have locally, Gazebo tries to download it on the fly. Depending on your internet connection and the size of the high-res textures, this can make the initial load feel like it's hanging.

> Note that we only incur this penalty once. Once a model is downloaded, it is cached in your local folder (usually ~/.gz/fuel or ~/.ignition/fuel). Subsequent loads should be near-instant.

#### Fix
To fix this issue, you can pre-download the necessary models and worlds from Gazebo Fuel before launching your simulation. This way, Gazebo won't need to download them during runtime, which should resolve the hanging issue.
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
