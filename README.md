# Why?
We believe that a single source of truth for robot models simplifies development and integration. By maintaining only one model — defined in URDF — we ensure consistency across platforms. This URDF serves as the common origin for both ROS 2 and Isaac Sim, enabling a streamlined workflow between simulation and real-world deployment. 

Just follow the [Quickstart guide](#quickstart) to get a simulation up and running — with full hardware interface support for ROS 2.

For more details, check out our ROSCon DE 2024 presentation: ["ROS2 Workflows in Omniverse Isaac Sim: Digital Twins & ros2_control"](https://roscon2024.de/presentations/S3_P3__ROS2_Workflows_in_Omniverse_Isaac_Sim_Digital_Twins_ros2_control.pdf). Also see our CASE 2025 paper: ["Simulation-to-Reality Hyperparameter Optimization of MPPI Controllers via Bayesian Optimization in NVIDIA Omniverse Isaac Sim"](https://proximityrobotics.github.io/OmniMPPI/).

# Proximity Isaac URDF Importer

The *PX Isaac Sim URDF Importer* builds on top of [Isaac Sim's URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/ext_isaacsim_asset_importer_urdf.html#isaac-sim-urdf-importer) and extends its functionality. This repository includes a sample script that illustrates a way to use the importer to create a scene. It also provides an example of how ROS 2 nodes can be used in Isaac Sim to control imported robots.

⚠️ Note: This README applies only to the isaac-4.2 branch (Isaac Sim version 4.2). If you're using the [*isaac-4.5*](https://gitlab.proximityrobotics.com/kathrin.alba/proximity-isaac-urdf-importer/tree/main) branch, please refer to its specific README.

## First Setup

- Clone this repository. 

- From the [Omniverse Container Installation Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html) make sure that all the requirements under ***Container Setup*** are met.

- Next, under ***Container Deployment*** follow steps ***1.*** *Setup and install the container prerequisites* through ***5.*** *Log in to NGS*. Skip steps 6., 7., and 8. and continue directly with step ***9.*** *Install the Omniverse Streaming Client from the Omniverse Launcher*. [Here](https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage) is a direct download link for the Omniverse Launcher AppImage. NVIDIA Omniverse Launcher will be deprecated on October 1, 2025. More info on the tools from the Omniverse Launcher can be found [here](https://developer.nvidia.com/omniverse/legacy-tools).

- **Before** continuing with step 10.:
    - Configure the scene description and robot description in the [*config.env*](config.env) file as described in [Configuration](#configuration). That section also describes configuring the `ROS_DOMAIN_ID` and which data, scripts, and humble workspace folder to mount.
    - Optionally extend your URDF or xacro file with elements described in [Isaac URDF](#isaac-urdf)
    - Build and run the docker container:
        ```bash
        ./start_docker.sh
        ```
        The working directory in the container should then be */isaac-sim*.
    - The ROS 2 Humble workspace is located at */isaac-sim/project/humble_ws* in the container. If you have packages in the workspace that you need, build it and source the environment.
        ```bash
        cd project/humble_ws
        colcon build
        . install/setup.bash
        ```
    - Check if you need to make some changes to the [simulation example script](scripts/run_simulation.py) as described in [Simulation Example](#simulation-example).
    - Start the simulation example with the alias `run_simulation` or with
        ```bash
        /isaac-sim/python.sh /isaac-sim/project/scripts/run_simulation.py
        ```
- Lastly, continue with step ***10.*** *Enter the IP address of the machine or instance running the Isaac Sim container and click on the Connect button to begin live streaming* from the Omniverse Container Installation Documentation.

## Usage
After the [First Setup](#first-setup) is done once, you can skip all the steps up to and including step 9 and continue with the steps "Before continuing with step 10.".

## Configuration
The simulation example starts the simulation with a scene and imports a robot from a URDF or xacro file into the scene.

### Scene Description and Robot Description
The simulation example automatically reads the environment variables `SCENE_DESCRIPTION` and `ROBOT_DESCRIPTION`. They can be specified in the [config.env](config.env) and will be set automatically when starting the docker container.

The `SCENE_DESCRIPTION` variable holds the path to a USD scene. If the variable value...
- ... is empty, a scene with only a ground plane and a dome light will be created automatically.
- ... starts with "http", the given USD file will be loaded. This is meant to be used for NVIDIA Omniverse Assets like the full warehouse scene *ht<span>tp://</span>omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Warehouse/full_warehouse.usd*.
- ... does not start with "http", it is expected to be a path relative to the *data/* folder (e.g. *scenes/my_scene.usd*) or an absolute path to the file in the ccontainer (e.g., **).

The `ROBOT_DESCRIPTION` variable holds the path to the robot description file. It can be a URDF or xacro file. If the variable value...
- ... is empty or does not end with ".urdf" or ".xacro", no robot is importet and the scene will remain as is.
- ... does end with ".urdf" or ".xacro", it is expected to be a path relative to the *data/* folder (e.g., *robots/my_robot.urdf*, or *robots/my_robot.xacro*) or an absolute path to the file in the container (e.g., */isaac-sim/project/humble_ws/src/my_description_package/urdf/my_robot.urdf*).

### Mounts and ROS Domain ID
In the [*start_docker.sh*](start_docker.sh) file four variables are defined as follows:
```sh
SRC_HOST="$(pwd)"/scripts
DATA_HOST="$(pwd)"/data
WS_HOST="$(pwd)"/humble_ws

DOMAIN_ID=0
```

The `DOMAIN_ID` is used as value for the `ROS_DOMAIN_ID` environment variable. 

The variables `SRC_HOST`, `DATA_HOST`, and `WS_HOST` hold paths to folders that will be mounted at */isaac-sim/project/scripts*, */isaac-sim/project/data*, and */isaac-sim/project/humble_ws* in the container.

```plaintext
/isaac-sim/  
├── project/ 
│   ├── scripts/
│   │   ├── run_simulation.py
│   │   └── ...
│   ├── data/
│   │   ├── robots/
│   │   └── scenes/
│   └── humble_ws/
│   │   ├── src/
│   │   └── ...
└── ... 
```

## Simulation Example
The simulation example script [run_simulation.py](scripts/run_simulation.py) 
- loads the USD scene defined with the `SCENE_DESCRIPTION` environment variable and imports the robot defined with the `ROBOT_DESCRIPTION` environment variable as described in [Configuration](#configuration),
- publishes the current simulation time every 0.1 s on a "clock" topic ([simulation_time_pub.py](scripts/simulation_time_pub.py)), and
- creates an articulation controller ([articulation_controller.py](scripts/articulation_controller.py)) that publishes the robot's current joint states on a "joint_states" topic and subscribes to a "joint_commands" topic to adjust the robot's joints.

The simulation example script can be executed with the `run_simulation` alias or with
```bash
/isaac-sim/python.sh /isaac-sim/project/scripts/run_simulation.py
```

### Command Line Arguments
The simulation example script accepts the following command line arguments:
<table style="border: none;">
    <tr>
        <td style="border: none;">--fix-base</td>
        <td style="border: none;">Fix the robot's root link.</td>
    </tr>
    <tr>
        <td style="border: none;">--xacro-mappings</td>
        <td style="border: none;">Mappings for the xacro robot description file in JSON format, e.g., '{"key": "value"}'.</td>
    </tr>
    <tr>
        <td style="border: none;">--joint-commands-topic</td>
        <td style="border: none;">The name for the joint commands topic.</td>
    </tr>
    <tr>
        <td style="border: none;">--joint-states-topic</td>
        <td style="border: none;">The name for the joint states topic.</td>
    </tr>
    <tr>
        <td style="border: none;">--viewport-dimensions</td>
        <td style="border: none;">The width and height of the Isaac Sim viewport.</td>
    </tr>
    <tr>
        <td style="border: none;">--window-dimensions</td>
        <td style="border: none;">The width and height of the Isaac Sim window.</td>
    </tr>
    <tr>
        <td style="border: none;">--default-damping</td>
        <td style="border: none;">The damping of the robot's joints if it is not specified in the URDF.</td>
    </tr>
    <tr>
        <td style="border: none;">--default-stiffness</td>
        <td style="border: none;">The stiffness of the robot's joints if it is not specified in the URDF.</td>
    </tr>
    <tr>
        <td style="border: none;">--robot-position</td>
        <td style="border: none;">The robot's position in the world as x, y, z coordinate.</td>
    </tr>
    <tr>
        <td style="border: none;">--robot-orientation</td>
        <td style="border: none;">The robot's orientation in the world as quaternion w, x, y, z.</td>
    </tr>
    <tr>
        <td style="border: none;">--scene-position</td>
        <td style="border: none;">The scenes's position in the world as x, y, z coordinate.</td>
    </tr>
    <tr>
        <td style="border: none;">--scene-orientation</td>
        <td style="border: none;">The scenes's orientation in the world as quaternion w, x, y, z.</td>
    </tr>
    <tr>
        <td style="border: none;">--scene-scale</td>
        <td style="border: none;">The scaling factor for the scene in x, y, and z direction.</td>
    </tr>
    <tr>
        <td style="border: none;">--draw-point-cloud</td>
        <td style="border: none;">Draw the point cloud of any LiDAR imported from the robot description file.</td>
    </tr>
    <tr>
        <td style="border: none;">--enable-gpu-dynamics</td>
        <td style="border: none;">Enable GPU dynamics. Required for particle systems.</td>
    </tr>
</table>


## Isaac URDF
URDF allows you to add damping and friction to a joint but it does not support stiffness / drive strength. With URDF you do not have an xml element that contains the info that you need to create a LiDAR sensor in Isaac Sim. Therefore, an `<isaac_drive_api>` tag for joints and tags to create LiDAR sensors have been added in this repository. You can add them to the URDF file as described in the following two sections and the [ProximityURDFImporter](scripts/proximity_urdf_importer.py) will then use them.

### URDF Tag for Joint Drive Gains in Isaac Sim
You can add the `<isaac_drive_api>` element to a joint in a URDF file as follows:
```xml
<joint name="my_joint" type="revolute">
    <origin xyz="0 0 1" rpy="0 0 3.1416"/>
    <parent link="link1"/>
    <child link="link2"/>
    <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
    <isaac_drive_api stiffness="1000000.0" damping="100000.0" friction="1.0" />
</joint>
```

If the `<isaac_drive_api>` element is not added to the joints in your URDF file, the original URDF's damping and friction values from the dynamics tag will be used, while stiffness / drive strength will be set to its default value. If damping and friction are not set in the original URDF, they will also be set to their default values.

The default values are:
- 1000000.0 for stiffness / drive strength
- 100000.0 for damping, and
- 0.0 for friction.

You can change the default values for stiffness and damping with the --default-damping and --default-stiffness command line arguments when running the simulation example script as described in the [Command Line Arguments](#command-line-arguments) section.

A tutorial on how to tune the joint drive gains can be found at this [Omniverse Tutorial for Advanced Joint Tuning](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_joint_tuning.html).

### URDF Tag for LiDAR Sensors in Isaac Sim
You can add the `<sensor>` element as a child to an `<isaac>` element to a robot in a URDF file to create a LiDAR sensor in Isaac Sim as follows:

```xml
<link name="lidar_sensor" />

<joint name="lidar_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 -1.5708 0" />
    <parent link="tool0" />
    <child link="lidar_sensor" />
    <axis xyz="0 0 0" />
</joint>

<isaac>
    <!-- sensor name must be name of link -->
    <sensor name="lidar_sensor" type="lidar">
        <topic>robot/lidar_sensor/points</topic>
        <config>Example_Rotary</config>
    </sensor>
</isaac>
```

Currently, only sensors of type "lidar" are handled in this repo. The `<sensor>` element must have two children `<topic>` and `<config>`. The `<topic>` element specifies the topic on which the point cloud data will be published. The `<config>` element specifies the name of the LiDAR configuration file. 

If the text content of the `<config>` element...
- ... does not end with ".json" (e.g., "Example_Rotary"), it is expected to be the name of a config that is provided by Isaac Sim. The Omniverse LiDAR config files can be found in the container at `/isaac-sim/exts/omni.isaac.sensor/data/lidar_configs/` and `/isaac-sim/extscache/omni.sensors.nv.common/data/lidar/`.
- ... ends with ".json" and starts with "/" (e.g., */isaac-sim/project/data/my_lidar_config.json*), it is assumed to be an absolute path to the LiDAR configuration file in the container. 
- ... ends with ".json" and does not start with "/" (e.g., *lidar_configs/my_config.json*), it is assumed to be a relative path to the LiDAR configuration file in the *data/* folder.

The name of the config file needs to be unique because Isaac Sim checks a list of folders for the filename only and does not use the whole path to determine which config file to use.

The available parameters for LiDAR configuration files are described at the [Omniverse Lidar Extension Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/omni_sensors_docs/lidar_extension.html).

