# Multivehicle Simulator

## Issues and fixes
### [ros_gz_sim]: Requesting list of world names.
When using the `ros_gz_sim` package, you may encounter an issue when requesting the list of world names. This happens because gazebo is trying to download some models from Gazebo Fuel.

Gazebo Fuel is an online database of models and worlds. When you launch a world that references an asset you don't have locally, Gazebo tries to download it on the fly. Depending on your internet connection and the size of the high-res textures, this can make the initial load feel like it's hanging.

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
