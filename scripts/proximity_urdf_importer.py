#
#  Copyright 2024 Proximity Robotics & Automation GmbH
#  
#  Licensed under the Apache License, Version 2.0 (the “License”);
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
# 
#        http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an “AS IS” BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# 

import os
import xacro
import carb
import numpy as np
import xml.etree.ElementTree as ET
from typing import Union, Sequence, Optional
from pathlib import Path

import omni.isaac.core
import omni.kit.widget.stage.export_utils
import omni.replicator.core as rep
from omni.importer.urdf import _urdf
from pxr import UsdPhysics, PhysxSchema, Usd

from lidar import LiDAR


class ProximityURDFImporter:

    def __init__(
        self,
        simulation_app,
        urdf_path: Optional[str] = None,
        xacro_path: Optional[str] = None,
        usd_path: Optional[str] = None,
        draw_point_cloud: bool = False,
    ) -> None:
        """The IsaacURDFImporter helps with importing robots from xacro/urdf to an Isaac Sim SimulationApp instance.

        :param simulation_app: The Isaac Sim Simulation Application.
        :type simulation_app: omni.isaac.kit.SimulationApp
        :param urdf_path: Path to the (desired) URDF file. Defaults to None.
        :type urdf_path: Optional[str], optional
        :param xacro_path: Path to the xacro file. Defaults to None.
        :type xacro_path: Optional[str], optional
        :param usd_path: Desired path to store the generated USD file of the robot. Defaults to None.
        :type usd_path: Optional[str], optional
        :param draw_point_cloud: Whether to draw the point cloud of LiDAR sensors. Defaults to False.
        :type draw_point_cloud: bool
        """
        self.simulation_app = simulation_app
        self.urdf_path = urdf_path
        self.xacro_path = xacro_path
        self.usd_path = usd_path
        self.draw_point_cloud = draw_point_cloud

    def import_robot_from_xacro(
        self,
        xacro_path: Optional[str] = None,
        urdf_path: Optional[str] = None,
        usd_path: Optional[str] = None,
        prim_path: Optional[str] = None,
        xacro_mappings: Optional[dict] = None,
        position: Union[Sequence[float], np.ndarray] = [0.0, 0.0, 0.0],
        orientation: Union[Sequence[float], np.ndarray] = [1.0, 0.0, 0.0, 0.0],
        fix_base: bool = False,
        default_drive_strength: float = 1000000.0,
        default_position_drive_damping: float = 100000.0,
    ) -> str:
        """Creates a URDF file from an xacro file and then imports the robot from the URDF file to the running Isaac Sim
         SimulationApp. In the process a USD file of the robot is created.

        :param xacro_path: Path to the xacro file. Defaults to None.
        :type xacro_path: Optional[str], optional
        :param urdf_path: Path where the URDF file will be created. Defaults to None.
        :type urdf_path: Optional[str], optional
        :param usd_path: Path where the USD file will be created. Defaults to None.
        :type usd_path: Optional[str], optional
        :param prim_path: Path to the prim in the stage. Defaults to None which will result in
         "World/Robots/<robot_name>".
        :type prim_path: Optional[str], optional
        :param xacro_mappings: Mappings for xacro processing the xacro file. Defaults to None.
        :type xacro_mappings: Optional[dict], optional
        :param position: Position xyz at which to spawn the robot. Defaults to [0.0, 0.0, 0.0].
        :type position: Union[Sequence[float], np.ndarray], optional
        :param orientation: Orientation as quaternion wxyz at which to spawn the robot. Defaults to [1.0, 0.0, 0.0, 0.0].
        :type orientation: Union[Sequence[float], np.ndarray], optional
        :param fix_base: Whether to fix the robot's base link. Defaults to False.
        :type fix_base: bool
        :param default_drive_strength: Default value for the drive strength if not specified otherwise in the URDF.
         Defaults to 1000000.0.
        :type default_drive_strength: float
        :param default_position_drive_damping: Default value for the drive damping if not specified otherwise in the
         URDF. Defaults to 100000.0.
        :type default_position_drive_damping: float
        :return: The prim path in the Isaac Sim stage at which the robot prim has been placed.
        :rtype: str
        """
        self.xacro_path = xacro_path or self.xacro_path
        self.urdf_path = urdf_path or self.urdf_path
        self.usd_path = usd_path or self.usd_path

        self.xacro_to_urdf(xacro_mappings=xacro_mappings)

        prim_path = self.import_robot_from_urdf(
            urdf_path=self.urdf_path,
            usd_path=self.usd_path,
            prim_path=prim_path,
            position=position,
            orientation=orientation,
            fix_base=fix_base,
            default_drive_strength=default_drive_strength,
            default_position_drive_damping=default_position_drive_damping,
        )

        return prim_path

    def import_robot_from_urdf(
        self,
        urdf_path: Optional[str] = None,
        usd_path: Optional[str] = None,
        prim_path: Optional[str] = None,
        position: Union[Sequence[float], np.ndarray] = [0.0, 0.0, 0.0],
        orientation: Union[Sequence[float], np.ndarray] = [1.0, 0.0, 0.0, 0.0],
        fix_base: bool = False,
        default_drive_strength: float = 1000000.0,
        default_position_drive_damping: float = 100000.0,
    ) -> str:
        """Imports a robot from a URDF file to the running Isaac Sim SimulationApp. In the process a USD file of the
         robot is created.

        :param urdf_path: Path to the URDF file. Defaults to None.
        :type urdf_path: Optional[str], optional
        :param usd_path: Path where the USD file will be created. Defaults to None.
        :type usd_path: Optional[str], optional
        :param prim_path: Path to the prim in the stage. Defaults to None which will result in
         "World/Robots/<robot_name>".
        :type prim_path: Optional[str], optional
        :param position: Position xyz at which to spawn the robot. Defaults to [0.0, 0.0, 0.0].
        :type position: Union[Sequence[float], np.ndarray], optional
        :param orientation: Orientation as quaternion wxyz at which to spawn the robot. Defaults to [1.0, 0.0, 0.0, 0.0].
        :type orientation: Union[Sequence[float], np.ndarray], optional
        :param fix_base: Whether to fix the robot's base link. Defaults to False.
        :type fix_base: bool
        :param default_drive_strength: Default value for the drive strength if not specified otherwise in the URDF.
         Defaults to 1000000.0.
        :type default_drive_strength: float
        :param default_position_drive_damping: Default value for the drive damping if not specified otherwise in the
         URDF. Defaults to 100000.0.
        :type default_position_drive_damping: float
        :return: The prim path in the Isaac Sim stage at which the robot prim has been placed.
        :rtype: str
        """
        # Define desired paths
        self.xacro_path = None
        self.urdf_path = urdf_path or self.urdf_path
        self.usd_path = usd_path or self.usd_path

        # Create USD from URDF
        old_articulation_root_path = self.urdf_to_usd(
            fix_base=fix_base,
            default_drive_strength=default_drive_strength,
            default_position_drive_damping=default_position_drive_damping,
        )

        # TODO what to do when old_articulation_root path empty (probably URDF Import failed)

        # Load robot into stage
        robot_name = old_articulation_root_path.split("/")[1]
        desired_prim_path = prim_path or "/World/Robots/" + robot_name
        prim_path = self.load_from_usd(
            prim_path=desired_prim_path,
            position=position,
            orientation=orientation,
        )

        self.adjust_articulation_root(
            prim_path=prim_path, old_articulation_root_path=old_articulation_root_path
        )
        self.apply_custom_joint_drive_gains(
            prim_path=prim_path,
            default_drive_strength=default_drive_strength,
            default_position_drive_damping=default_position_drive_damping,
        )
        self.create_sensors_from_urdf(prim_path=prim_path)

        return prim_path

    def xacro_to_urdf(
        self,
        xacro_path: Optional[str] = None,
        urdf_path: Optional[str] = None,
        xacro_mappings: Optional[str] = None,
    ) -> None:
        """Process xacro file and save as URDF.

        :param xacro_path: Path to the xacro file. Defaults to None.
        :type xacro_path: Optional[str], optional
        :param urdf_path: Path where the URDF file will be saved. Defaults to None.
        :type urdf_path: Optional[str], optional
        :param xacro_mappings: Mappings for xacro processing the xacro file. Defaults to None.
        :type xacro_mappings: Optional[str], optional
        """
        self.xacro_path = xacro_path or self.xacro_path
        self.urdf_path = urdf_path or self.urdf_path

        # Load robot description from xacro file
        assert os.path.isfile(self.xacro_path)
        xacro_file = xacro.process_file(self.xacro_path, mappings=xacro_mappings)
        robot_description = xacro_file.toprettyxml(indent="    ")  # TODO 2 or 4 spaces?

        # Make sure urdf file can be created correctly
        urdf_dirpath, urdf_filename = os.path.split(self.urdf_path)
        assert os.path.splitext(urdf_filename)[1] == ".urdf"
        if not os.path.exists(urdf_dirpath):
            os.makedirs(urdf_dirpath)

        # Save robot description in urdf file
        with open(self.urdf_path, "w") as f:
            f.write(robot_description)

    def urdf_to_usd(
        self,
        urdf_path: Optional[str] = None,
        usd_path: Optional[str] = None,
        fix_base: bool = False,
        default_drive_strength: float = 1000000.0,
        default_position_drive_damping: float = 100000.0,
    ) -> str:
        """Import a robot from a URDF file using the Isaac Sim URDF Importer. The importer automatically creates a USD
         file for the imported robot.

        :param urdf_path: Path to the URDF file. Defaults to None.
        :type urdf_path: Optional[str], optional
        :param usd_path: Path where the USD file will be saved. Defaults to None.
        :type usd_path: Optional[str], optional
        :param fix_base: Whether to fix the robot's base link. Defaults to False.
        :type fix_base: bool
        :param default_drive_strength: Default value for the drive strength if not specified otherwise in the URDF.
         Defaults to 1000000.0.
        :type default_drive_strength: float
        :param default_position_drive_damping: Default value for the drive damping if not specified otherwise in the
         URDF. Defaults to 100000.0.
        :type default_position_drive_damping: float
        :return: The prim path to the articulation root prim in the created USD file. This prim path should usually be
         "/<robot_name>/<first_link_name>".
        :rtype: str
        """
        self.urdf_path = urdf_path or self.urdf_path
        self.usd_path = usd_path or self.usd_path

        _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.set_merge_fixed_joints(False)
        import_config.set_replace_cylinders_with_capsules(False)
        import_config.set_convex_decomp(True)
        import_config.set_fix_base(fix_base)
        import_config.set_import_inertia_tensor(True)
        import_config.set_distance_scale(1.0)
        import_config.set_density(0.0)
        import_config.set_default_drive_type(
            _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        )
        import_config.set_default_drive_strength(default_drive_strength)
        import_config.set_default_position_drive_damping(default_position_drive_damping)
        import_config.set_self_collision(False)
        import_config.set_up_vector(0, 0, 1)
        import_config.set_make_default_prim(True)
        import_config.set_parse_mimic(True)
        import_config.set_create_physics_scene(False)
        import_config.set_collision_from_visuals(False)
        import_config.set_override_joint_dynamics(False)

        _, old_articulation_root_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=self.urdf_path,
            import_config=import_config,
            get_articulation_root=True,
            dest_path=self.usd_path,
        )

        return old_articulation_root_path

    def load_from_usd(
        self,
        prim_path: str,
        usd_path: Optional[str] = None,
        position: list[float] = [0.0, 0.0, 0.0],
        orientation: list[float] = [1.0, 0.0, 0.0, 0.0],
    ) -> str:
        """Loads the robot from the USD file to the Isaac Sim stage.

        :param prim_path: Prim path at which the robot is desired to be created in the stage. If a prim already exists
         at this path, the last part of the path will be numbered incrementally until a unique path is found.
        :type prim_path: str
        :param usd_path: USD path from where to load the robot. Defaults to None.
        :type usd_path: Optional[str], optional
        :param position: Position xyz where to spawn the robot. Defaults to [0.0, 0.0, 0.0].
        :type position: list[float], optional
        :param orientation: Orientation as quaternion wxyz in which to spawn the robot. Defaults to [1.0, 0.0, 0.0, 0.0].
        :type orientation: list[float], optional
        :return: The actual prim path of the robot prim.
        :rtype: str
        """
        self.usd_path = usd_path or self.usd_path

        prim_path = omni.usd.get_stage_next_free_path(
            self.simulation_app.context.get_stage(),
            prim_path,
            False,
        )
        omni.isaac.core.utils.prims.create_prim(
            prim_path,
            "Xform",
            usd_path=self.usd_path,
            position=position,
            orientation=orientation,
        )
        return prim_path

    def adjust_articulation_root(
        self, prim_path: str, old_articulation_root_path: str
    ) -> None:
        """Adjusts the articulation root API. Per default the prim representing the first link in the URDF file (usually
         "base_link") is assigned the articulation root API, but the parent prim should be the articulation root.

        :param prim_path: The path to the robot prim in the Isaac Sim stage
        :type prim_path: str
        :param old_articulation_root_path: The path to the articulation root in the USD file created by the Isaac Sim
         URDF importer. It contains the robot name.
        :type old_articulation_root_path: str
        """
        # Get current stage
        stage = self.simulation_app.context.get_stage()

        # Remove current articulation root
        current_articulation_root = (
            prim_path + "/" + "/".join(old_articulation_root_path.split("/")[2:])
        )
        stage.GetPrimAtPath(current_articulation_root).RemoveAPI(
            UsdPhysics.ArticulationRootAPI
        )

        # Add new articulation root
        articulation_root = stage.GetPrimAtPath(prim_path)
        UsdPhysics.ArticulationRootAPI.Apply(articulation_root)
        root_api = PhysxSchema.PhysxArticulationAPI.Apply(articulation_root)
        root_api.CreateEnabledSelfCollisionsAttr().Set(False)
        root_api.CreateSolverVelocityIterationCountAttr().Set(16)

        self.simulation_app.update()

    def set_gains_attributes(
        self, joint_prim: Usd.Prim, api_attribs: dict, drive_type: str
    ) -> None:
        """Sets stiffness, damping and friction attributes for a given joint prim with a specific drive type.

        :param joint_prim: The joint prim.
        :type joint_prim: Usd.Prim
        :param api_attribs: The drive gains attributes.
        :type api_attribs: dict
        :param drive_type: The joint drive type.
        :type drive_type: str
        """
        drive_api = UsdPhysics.DriveAPI.Get(
            joint_prim,
            drive_type,
        )
        joint_api = PhysxSchema.PhysxJointAPI(joint_prim)

        if "stiffness" in api_attribs:
            drive_api.CreateStiffnessAttr().Set(float(api_attribs["stiffness"]))
        if "damping" in api_attribs:
            drive_api.CreateDampingAttr().Set(float(api_attribs["damping"]))
        if "friction" in api_attribs:
            joint_api.CreateJointFrictionAttr().Set(float(api_attribs["friction"]))

    def apply_custom_joint_drive_gains(
        self,
        prim_path: str,
        urdf_path: Optional[str] = None,
        default_drive_strength: float = 1000000.0,
        default_position_drive_damping: float = 100000.0,
    ) -> None:
        """Reads the isaac_drive_api tag from the URDF file and applies its specified drive gains.

        :param prim_path: Path to the robot prim in the Isaac Sim stage.
        :type prim_path: str
        :param urdf_path: Path to the URDF file. Defaults to None.
        :type urdf_path: Optional[str], optional
        :param default_drive_strength: Default value for the drive strength if not specified otherwise in the URDF.
         Defaults to 1000000.0.
        :type default_drive_strength: float
        :param default_position_drive_damping: Default value for the drive damping if not specified otherwise in the
         URDF. Defaults to 100000.0.
        :type default_position_drive_damping: float
        """
        self.urdf_path = urdf_path or self.urdf_path

        # Get current stage
        stage = self.simulation_app.context.get_stage()

        # Set drive damping, stiffness, and joint friction
        urdf_root = ET.parse(self.urdf_path).getroot()
        drive_type = {
            "revolute": "angular",
            "continuous": "angular",
            "prismatic": "linear",
            "fixed": "fixed",
        }
        for joint in urdf_root.findall("./joint"):
            isaac_drive_api = joint.find("./isaac_drive_api")
            dynamics = joint.find("./dynamics")
            if isaac_drive_api is not None:
                if joint.attrib["type"] in drive_type:
                    parent = joint.find("./parent")
                    api_attribs = isaac_drive_api.attrib
                    joint_prim = stage.GetPrimAtPath(
                        f"{prim_path}/{parent.attrib['link']}/{joint.attrib['name']}"
                    )
                    self.set_gains_attributes(
                        joint_prim=joint_prim,
                        api_attribs=api_attribs,
                        drive_type=drive_type[joint.attrib["type"]],
                    )
                else:
                    carb.log_warn(
                        f"Drive type {joint.attrib['type']} currently not supported"
                        + " for isaac_drive_api tag on URDF import."
                    )
            elif dynamics is None:
                parent = joint.find("./parent")
                joint_prim = stage.GetPrimAtPath(
                    f"{prim_path}/{parent.attrib['link']}/{joint.attrib['name']}"
                )
                self.set_gains_attributes(
                    joint_prim=joint_prim,
                    api_attribs={
                        "stiffness": default_drive_strength,
                        "damping": default_position_drive_damping,
                        "friction": 0.0,
                    },
                    drive_type=drive_type[joint.attrib["type"]],
                )

    def get_sensor_prim_path(self, sensor: ET.Element, prim_path: str) -> str:
        """Returns the path to a sensor prim in the Isaac Sim stage.

        :param sensor: The sensor object read from the URDF file.
        :type sensor: ET.Element
        :param prim_path: Path to the robot prim in the Isaac Sim stage.
        :type prim_path: str
        :return: The path to the sensor prim in the Isaac Sim stage.
        :rtype: str
        """
        sensor_name = sensor.attrib["name"]
        sensor_path = f"{prim_path}/{sensor_name}"
        return sensor_path

    def create_sensor_prim(
        self,
        sensor_path: str,
        config_name: str,
    ) -> LiDAR:
        """Creates an RTX LiDAR sensor prim at the given path in the Isaac Sim stage.

        :param sensor_path: Path at which to create the sensor.
        :type sensor_path: str
        :param config_name: Name of the LiDAR config.
        :type config_name: str
        :return: The sensor prim.
        :rtype: Usd.Prim
        """
        lidar_sensor = LiDAR(
            lidar_config_name=config_name,
            prim_path=sensor_path + "/" + sensor_path.split("/")[-1] + "_sensor",
        )
        return lidar_sensor

    def add_lidar_config_path(self, config_file_path: str) -> str:
        """Add the path of the LiDAR config to the lidar profile base folder so that it canf be found when creating a
         LiDAR sensor in Isaac Sim.

        :param config_file_path: Path to the config file.
        :type config_file_path: str
        :return: Name of the config file without the file extension.
        :rtype: str
        """
        config_dirname, config_filename = os.path.split(config_file_path)
        config_name = os.path.splitext(config_filename)[0]

        carb_settings = carb.settings.get_settings()
        lidar_config_folders = carb_settings.get(
            "/app/sensors/nv/lidar/profileBaseFolder"
        )
        if not config_dirname in lidar_config_folders:
            lidar_config_folders.append(config_dirname)
            carb_settings.set(
                "/app/sensors/nv/lidar/profileBaseFolder", lidar_config_folders
            )
        return config_name

    def create_sensors_from_urdf(
        self, prim_path: str, urdf_path: Optional[str] = None
    ) -> None:
        self.urdf_path = urdf_path or self.urdf_path

        urdf_root = ET.parse(self.urdf_path).getroot()
        for sensor in urdf_root.findall("./isaac/sensor"):
            if sensor.attrib["type"] == "lidar":
                # TODO skip sensors that do not have all required tags?
                # Add lidar config path
                config = sensor.find("./config")

                # urdf_dir = os.path.dirname(self.urdf_path)
                config_name = config.text
                if config_name.endswith(".json"):
                    if config_name[0] == "/":
                        config_file_path = config_name
                    else:
                        # config_file_path = os.path.abspath(
                        #     os.path.join(urdf_dir, config.text)
                        # )
                        config_file_path = os.path.abspath(
                            os.path.join(
                                "/isaac-sim/project/data/", config.text
                            )
                        )

                    config_name = self.add_lidar_config_path(
                        config_file_path=config_file_path
                    )

                # Create lidar sensor
                sensor_path = self.get_sensor_prim_path(
                    sensor=sensor, prim_path=prim_path
                )
                lidar_sensor = self.create_sensor_prim(
                    sensor_path=sensor_path,
                    config_name=config_name,
                )

                # Publish sensor data
                topic = sensor.find("./topic")
                topic_name = topic.text
                lidar_sensor.attach_point_cloud_publisher(
                    topic_name=topic_name, frame_id=sensor_path.split("/")[-1]
                )
                if self.draw_point_cloud:
                    lidar_sensor.draw_point_cloud()
            else:
                carb.log_warn(
                    f"Sensor type '{sensor.attrib['type']}' currently not supported to import from URDF. File '{urdf_path}'."
                )

    def replace_package_paths(
        self, replace_paths: dict, urdf_path: Optional[str] = None
    ) -> None:
        """In the URDF file replace all package://<package_name> and similar references that cannot be resolved by the
         Isaac Sim URDF importer.

        :param replace_paths: A dict with the string to be replaced (e.g. "package://my_robot") as key and the
         respective absolute path to the package or relative path from the URDF file to the package as value.
        :type replace_paths: dict
        :param urdf_path: Path to the URDF file. If None, the value from self.urdf_path will be used. Defaults to None.
        :type urdf_path: Optional[str], optional
        """
        self.urdf_path = urdf_path or self.urdf_path
        p = Path(self.urdf_path)
        rename_path = f"{Path.joinpath(p.parent, p.stem)}_{p.suffix}"
        try:
            os.rename(self.urdf_path, rename_path)
            with open(rename_path, "rt") as fin:
                with open(self.urdf_path, "wt") as fout:
                    for line in fin:
                        for package_path in replace_paths:
                            line = line.replace(
                                package_path, replace_paths[package_path]
                            )
                        fout.write(line)
            os.remove(rename_path)
        except Exception as e:
            print(e)
            os.rename(rename_path, self.urdf_path)
