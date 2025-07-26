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

from __future__ import annotations

import os
import uuid
import json
import xml.etree.ElementTree as ET
from typing import Union, Sequence, Optional

import numpy as np

import xacro

import carb
import isaacsim.core
import omni.kit.commands
from isaacsim.asset.importer.urdf import _urdf

# Somehow isaacsim.core.utils.articulations.move_articulation_root(...) in code not working:
#  "module 'isaacsim.core.utils' has no attribute 'articulations'"
from isaacsim.core.utils.articulations import move_articulation_root

# Same here: "module 'isaacsim.core.utils' has no attribute 'physics'"
from isaacsim.core.utils.physics import get_rigid_body_enabled

from pxr import PhysxSchema

from sensors.lidar import Lidar
from sensors.camera import Camera
from articulation import Articulation


class ProximityURDFImporter:

    def __init__(
        self,
        simulation_app: isaacsim.SimulationApp,
        urdf_path: Optional[str] = None,
        xacro_path: Optional[str] = None,
        usd_path: Optional[str] = None,
        draw_point_cloud: bool = False,
    ) -> None:
        """The IsaacURDFImporter helps with importing robots from xacro/URDF to an Isaac Sim SimulationApp instance.

        :param simulation_app: The Isaac Sim Simulation Application.
        :type simulation_app: isaacsim.SimulationApp
        :param urdf_path: Path to the (desired) URDF file. Defaults to None.
        :type urdf_path: Optional[str], optional
        :param xacro_path: Path to the xacro file. Defaults to None.
        :type xacro_path: Optional[str], optional
        :param usd_path: Desired path to store the generated USD file of the robot. Defaults to None.
        :type usd_path: Optional[str], optional
        :param draw_point_cloud: Whether to draw the point cloud of LiDAR and Camera sensors. Defaults to False.
        :type draw_point_cloud: bool
        """
        self.simulation_app = simulation_app
        self.urdf_path = urdf_path
        self.xacro_path = xacro_path
        self.usd_path = usd_path
        self.draw_point_cloud = draw_point_cloud
        self.robot_name = None

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
    ) -> tuple[Articulation, str]:
        """Creates a URDF file from an xacro file and then imports the robot from the URDF file to the running Isaac Sim
         SimulationApp. In the process a USD file of the robot is created.

        :param xacro_path: Path to the xacro file. Defaults to None.
        :type xacro_path: Optional[str], optional
        :param urdf_path: Path where the URDF file will be created. Defaults to None.
        :type urdf_path: Optional[str], optional
        :param usd_path: Path where the USD file will be created. Defaults to None.
        :type usd_path: Optional[str], optional
        :param prim_path: Path to the prim in the stage. Is set to "World/Robots/<robot_name>" if None. Defaults to
         None.
        :type prim_path: Optional[str], optional
        :param xacro_mappings: Mappings for xacro processing the xacro file. Defaults to None.
        :type xacro_mappings: Optional[dict], optional
        :param position: Position xyz at which to spawn the robot. Defaults to [0.0, 0.0, 0.0].
        :type position: Union[Sequence[float], np.ndarray], optional
        :param orientation: Orientation as quaternion wxyz at which to spawn the robot. Defaults to [1.0, 0.0, 0.0, 0.0].
        :type orientation: Union[Sequence[float], np.ndarray], optional
        :param fix_base: Whether to fix the robot's base link. Defaults to False.
        :type fix_base: bool
        :param default_drive_strength: Default value for the drive strength if not specified otherwise in the URDF file.
         Defaults to 1000000.0.
        :type default_drive_strength: float
        :param default_position_drive_damping: Default value for the drive damping if not specified otherwise in the
         URDF file. Defaults to 100000.0.
        :type default_position_drive_damping: float
        :return: The robot's prim path in the stage and the prim path to the robot's root link.
        :rtype: tuple[str, str]
        """
        self.xacro_path = xacro_path or self.xacro_path
        self.urdf_path = urdf_path or self.urdf_path
        self.usd_path = usd_path or self.usd_path

        self.xacro_to_urdf(xacro_mappings=xacro_mappings)

        articulation, odom_prim_path = self.import_robot_from_urdf(
            urdf_path=self.urdf_path,
            usd_path=self.usd_path,
            prim_path=prim_path,
            position=position,
            orientation=orientation,
            fix_base=fix_base,
            default_drive_strength=default_drive_strength,
            default_position_drive_damping=default_position_drive_damping,
        )

        return articulation, odom_prim_path

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
    ) -> tuple[Articulation, str]:
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
        :return: The robot's prim path in the stage and the prim path to the robot's root link.
        :rtype: tuple[str, str]
        """
        # Define desired paths
        self.xacro_path = None
        self.urdf_path = urdf_path or self.urdf_path
        self.usd_path = usd_path or self.usd_path

        # Create USD from URDF
        self.robot_name, original_root_link_name, topics = self.urdf_to_usd(
            fix_base=fix_base,
            default_drive_strength=default_drive_strength,
            default_position_drive_damping=default_position_drive_damping,
        )

        # Load robot into stage
        desired_prim_path = prim_path or "/World/Robots/" + self.robot_name
        prim_path = self.load_from_usd(
            prim_path=desired_prim_path,
            position=position,
            orientation=orientation,
        )

        self.adjust_articulation_root(prim_path=prim_path)
        self.create_sensors_from_urdf(robot_prim_path=prim_path)

        # Get prim path to the robot's root link
        predicate = lambda path: not isaacsim.core.utils.prims.is_prim_path_valid(
            f"{path}/{original_root_link_name}"
        )
        first_matching_prim = isaacsim.core.utils.prims.get_first_matching_child_prim(
            prim_path, predicate
        )
        if first_matching_prim is not None:
            root_link_path = isaacsim.core.utils.prims.get_prim_path(
                first_matching_prim
            )
        else:
            root_link_path = prim_path + "/base_link"

        # Get joint_state_topic and joint_command_topic if available in URDF file
        joint_states_topic_name, joint_commands_topic_name = topics

        articulation = Articulation(
            prim_paths_expr=prim_path,
            name=self.robot_name + "_" + str(uuid.uuid4()),
            joint_states_topic_name=joint_states_topic_name,
            joint_commands_topic_name=joint_commands_topic_name,
        )

        return articulation, root_link_path

    def xacro_to_urdf(
        self,
        xacro_path: Optional[str] = None,
        urdf_path: Optional[str] = None,
        xacro_mappings: Optional[str] = None,
    ) -> None:
        """Process xacro file and save as URDF file.

        :param xacro_path: Path to the xacro file. If it is None, the xacro_path instance variable is used. Defaults to
         None.
        :type xacro_path: Optional[str], optional
        :param urdf_path: Path where the URDF file will be saved. If it is None, the urdf_path instance variable is
         used. Defaults to None.
        :type urdf_path: Optional[str], optional
        :param xacro_mappings: Mappings for xacro processing the xacro file. Defaults to None.
        :type xacro_mappings: Optional[str], optional
        """
        self.xacro_path = xacro_path or self.xacro_path
        self.urdf_path = urdf_path or self.urdf_path

        # Load robot description from xacro file
        assert os.path.isfile(self.xacro_path)
        xacro_file = xacro.process_file(self.xacro_path, mappings=xacro_mappings)
        robot_description = xacro_file.toprettyxml(indent="    ")

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
    ) -> tuple[str, str]:
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
        :return: The robot name and the name of the robot's root link.
        :rtype: tuple[str, str]
        """
        # Get desired paths
        self.urdf_path = urdf_path or self.urdf_path
        self.usd_path = usd_path or self.usd_path

        # Create the URDF import config
        import_config = ProximityURDFImporter.get_import_config(
            fix_base=fix_base,
            default_drive_strength=default_drive_strength,
            default_position_drive_damping=default_position_drive_damping,
        )

        # Parse the URDF file
        _, robot_model = omni.kit.commands.execute(
            "URDFParseFile", urdf_path=self.urdf_path, import_config=import_config
        )

        # Adjust the joint drive properties if the "isaac_drive_api" element is given in the URDF file
        urdf_root = ET.parse(self.urdf_path).getroot()
        robot_model = self.adjust_joint_drive_properties(
            urdf_root=urdf_root, urdf_robot=robot_model
        )

        # Import the robot from the parsed and adapted URDF robot model
        omni.kit.commands.execute(
            "URDFImportRobot",
            urdf_path=self.urdf_path,
            urdf_robot=robot_model,
            import_config=import_config,
            dest_path=self.usd_path,
        )

        joint_state_topic = None
        joint_command_topic = None
        for child in urdf_root.findall(".//ros2_control/hardware/param"):
            if child.attrib["name"] == "joint_commands_topic":
                joint_command_topic = child.text
            if child.attrib["name"] == "joint_states_topic":
                joint_state_topic = child.text

        return (
            robot_model.name,
            robot_model.root_link,
            (joint_state_topic, joint_command_topic),
        )

    @staticmethod
    def get_import_config(
        fix_base: bool = False,
        default_drive_strength: float = 1000000.0,
        default_position_drive_damping: float = 100000.0,
    ) -> _urdf.ImportConfig:
        """Create a URDF import config for the Isaac Sim URDF Importer.

        :param fix_base: Whether to fix the robots base link. Defaults to False.
        :type fix_base: bool, optional
        :param default_drive_strength: Default value for the drive strength if not specified otherwise in the URDF.
         Defaults to 1000000.0.
        :type default_drive_strength: float
        :param default_position_drive_damping: Default value for the drive damping if not specified otherwise in the
         URDF. Defaults to 100000.0.
        :type default_position_drive_damping: float
        :return: The import config.
        :rtype: isaacsim.asset.importer.urdf._urdf.ImportConfig
        """
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
        )  # JOINT_DRIVE_POSITION, JOINT_DRIVE_VELOCITY
        import_config.set_default_drive_strength(default_drive_strength)
        import_config.set_default_position_drive_damping(default_position_drive_damping)
        import_config.set_self_collision(False)
        import_config.set_up_vector(0, 0, 1)
        import_config.set_make_default_prim(True)
        import_config.set_parse_mimic(True)
        import_config.set_create_physics_scene(False)
        import_config.set_collision_from_visuals(False)
        # import_config.set_override_joint_dynamics(False)
        return import_config

    def load_from_usd(
        self,
        prim_path: str,
        usd_path: Optional[str] = None,
        position: list[float] = [0.0, 0.0, 0.0],
        orientation: list[float] = [1.0, 0.0, 0.0, 0.0],
    ) -> str:
        """Loads the robot from the USD file to the Isaac Sim stage.

        :param prim_path: Desired path for the robot prim in the stage. If a prim already exists at this path, the last
         part of the path will be numbered incrementally until a unique path is found.
        :type prim_path: str
        :param usd_path: USD path from where to load the robot. If it is None, the usd_path instance variable is used.
         Defaults to None.
        :type usd_path: Optional[str], optional
        :param position: Position xyz where to spawn the robot. Defaults to [0.0, 0.0, 0.0].
        :type position: list[float], optional
        :param orientation: Orientation as quaternion wxyz in which to spawn the robot. Defaults to
         [1.0, 0.0, 0.0, 0.0].
        :type orientation: list[float], optional
        :return: The actual prim path of the robot prim.
        :rtype: str
        """
        self.usd_path = usd_path or self.usd_path

        # Get a unique prim path
        prim_path = omni.usd.get_stage_next_free_path(
            self.simulation_app.context.get_stage(),
            prim_path,
            False,
        )

        # Create the robot prim in the stage
        isaacsim.core.utils.prims.create_prim(
            prim_path,
            "Xform",
            usd_path=self.usd_path,
            position=position,
            orientation=orientation,
        )
        return prim_path

    def adjust_articulation_root(self, prim_path: str) -> None:
        """Adjusts the articulation root API. Per default the prim representing the robot's root link has the
         articulation root API, but the parent prim should be the articulation root.

        :param prim_path: The path to the robot prim in the Isaac Sim stage
        :type prim_path: str
        """
        # TODO Find out if / when this is still necessary with Isaac Sim 4.5
        articulation_root_prim_path = (
            isaacsim.core.utils.prims.get_articulation_root_api_prim_path(prim_path)
        )
        new_articulation_root = isaacsim.core.utils.prims.get_prim_at_path(prim_path)

        move_articulation_root(
            isaacsim.core.utils.prims.get_prim_at_path(articulation_root_prim_path),
            new_articulation_root,
        )

        root_api = PhysxSchema.PhysxArticulationAPI.Apply(new_articulation_root)
        root_api.CreateEnabledSelfCollisionsAttr().Set(False)
        root_api.CreateSolverVelocityIterationCountAttr().Set(16)

    def adjust_joint_drive_properties(
        self, urdf_root: ET.Element, urdf_robot: _urdf.UrdfRobot
    ) -> _urdf.UrdfRobot:
        """Adjust the joints' properties if the "isaac_drive_api" element is given in the URDF file.

        :param urdf_root: The URDF file's content.
        :type urdf_root: xml.etree.ElementTree.Element
        :param urdf_robot: The robot object created by the Isaac Sim URDF File Parser.
        :type urdf_robot: isaacsim.asset.importer.urdf._urdf.UrdfRobot
        :return: The robot after the joints' properties have been adjusted.
        :rtype: isaacsim.asset.importer.urdf._urdf.UrdfRobot
        """
        for joint in urdf_root.findall("./joint"):
            isaac_drive_api = joint.find("./isaac_drive_api")
            if isaac_drive_api is not None:
                api_attribs = isaac_drive_api.attrib
                urdf_joint = urdf_robot.joints[joint.attrib["name"]]

                # Set joint friction
                if "friction" in api_attribs:
                    urdf_joint.dynamics.set_friction(float(api_attribs["friction"]))

                # Set position or velocity drive
                if "target_type" in api_attribs:
                    target_type = api_attribs["target_type"]
                    if "position" in target_type:
                        urdf_joint.drive.set_target_type(
                            _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
                        )
                    elif "velocity" in target_type:
                        urdf_joint.drive.set_target_type(
                            _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
                        )
                    else:
                        carb.log_warn(
                            f"Target type {target_type} for joint '{joint.attrib['name']}' from "
                            + f"'{urdf_robot.name}' URDF is not supported for the joint drive. "
                            + f"It must be 'position' or 'velocity'."
                        )

                # Set force or acceleration drive
                if "drive_type" in api_attribs:
                    drive_type = api_attribs["drive_type"]
                    if "force" in drive_type:
                        urdf_joint.drive.set_drive_type(
                            _urdf.UrdfJointDriveType.JOINT_DRIVE_FORCE
                        )
                    elif "acceleration" in drive_type:
                        urdf_joint.drive.set_drive_type(
                            _urdf.UrdfJointDriveType.JOINT_DRIVE_ACCELERATION
                        )
                    else:
                        carb.log_warn(
                            f"Drive type {drive_type} for joint '{joint.attrib['name']}' from "
                            + f"'{urdf_robot.name}' URDF is not supported for the joint drive. "
                            + f"It must be 'force' or 'acceleration'."
                        )

                # Set damping and drive strength from explicit attributes or natural frequency and damping ratio
                if "strength" in api_attribs or "damping" in api_attribs:
                    if "strength" in api_attribs:
                        urdf_joint.drive.set_strength(float(api_attribs["strength"]))
                    if "damping" in api_attribs:
                        urdf_joint.drive.set_damping(float(api_attribs["damping"]))
                elif (
                    "natural_frequency" in api_attribs or "damping_ratio" in api_attribs
                ):
                    if "natural_frequency" in api_attribs:
                        if "damping_ratio" in api_attribs:
                            new_damping_ratio = float(api_attribs["damping_ratio"])
                        else:
                            new_damping_ratio = None
                        urdf_joint = (
                            ProximityURDFImporter.get_joint_updated_natural_frequency(
                                new_value=float(api_attribs["natural_frequency"]),
                                urdf_robot=urdf_robot,
                                urdf_joint=urdf_joint,
                                damping_ratio=new_damping_ratio,
                            )
                        )
                    elif "damping_ratio" in api_attribs:
                        urdf_joint = (
                            ProximityURDFImporter.get_joint_updated_damping_ratio(
                                urdf_joint=urdf_joint,
                                new_value=float(api_attribs["damping_ratio"]),
                            )
                        )
                urdf_robot.joints[joint.attrib["name"]] = urdf_joint
        return urdf_robot

    @staticmethod
    def get_joint_updated_natural_frequency(
        new_value: float,
        urdf_robot: _urdf.UrdfRobot,
        urdf_joint: _urdf.UrdfJoint,
        damping_ratio: float = None,
    ) -> _urdf.UrdfJoint:
        """Calculate joint drive stiffness and damping from a given natural frequency and damping ratio the same way it
         is done in the Isaac Sim URDF Importer Extension:

         K = m * f^2, D = 2 * r * f * m
         where f is the natural frequency, r is the damping ratio, and m the total equivalent inertia at the joint.

         The damping ratio is such that
         r = 1.0 is a critically damped system,
         r < 1.0 is underdamped, and
         r > 1.0 is overdamped.

        :param new_value: Value for the natural frequency.
        :type new_value: float
        :param urdf_robot: The articulation object created by the Isaac Sim URDF File Parser.
        :type urdf_robot: isaacsim.asset.importer.urdf._urdf.UrdfRobot
        :param urdf_joint: The joint from the articulation object that is having its joint drive gains adjusted.
        :type urdf_joint: isaacsim.asset.importer.urdf._urdf.UrdfJoint
        :param damping_ratio: The damping ratio. Defaults to None.
        :type damping_ratio: float, optional
        :return: The joint with adjusted joint drive gains.
        :rtype: isaacsim.asset.importer.urdf._urdf.UrdfJoint
        """
        # Only continue if value changed
        if new_value == urdf_joint.drive.natural_frequency:
            return urdf_joint

        urdf_joint.drive.natural_frequency = new_value

        # Compute drive strength and only continue if it changed
        strength = _urdf.acquire_urdf_interface().compute_natural_stiffness(
            urdf_robot, urdf_joint.name, new_value
        )
        if strength == urdf_joint.drive.strength:
            return urdf_joint

        urdf_joint.drive.strength = strength

        # Only apply damping for position drive joint
        if (
            urdf_joint.drive.target_type
            == _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        ):
            damping_ratio = (
                urdf_joint.drive.damping_ratio
                if damping_ratio is None
                else damping_ratio
            )
            urdf_joint = ProximityURDFImporter.get_joint_updated_damping_ratio(
                urdf_joint=urdf_joint, new_value=damping_ratio
            )

        return urdf_joint

    @staticmethod
    def get_joint_updated_damping_ratio(
        urdf_joint: _urdf.UrdfJoint, new_value: float
    ) -> _urdf.UrdfJoint:
        """Update the joint drive damping depending on a given damping ratio and the natural frequency.

        :param urdf_joint: The joint to be updated.
        :type urdf_joint: _urdf.UrdfJoint
        :param new_value: The new damping ratio.
        :type new_value: float
        :return: The joint with its updated drive damping.
        :rtype: _urdf.UrdfJoint
        """
        m_eq = 1
        if urdf_joint.drive.drive_type == _urdf.UrdfJointDriveType.JOINT_DRIVE_FORCE:
            m_eq = urdf_joint.inertia

        urdf_joint.drive.damping_ratio = new_value

        urdf_joint.drive.damping = (
            2
            * m_eq
            * urdf_joint.drive.natural_frequency
            * urdf_joint.drive.damping_ratio
        )

        return urdf_joint

    @staticmethod
    def get_sensor_prim_path(
        sensor: ET.Element, robot_prim_path: str, sensor_type: str
    ) -> tuple[str, str]:
        """Returns the path for the given sensor prim in the Isaac Sim stage and the sensor's name.

        :param sensor: The sensor object read from the URDF file.
        :type sensor: ET.Element
        :param prim_path: Path to the robot prim in the Isaac Sim stage.
        :type prim_path: str
        :return: The path for the sensor prim in the Isaac Sim stage and the sensor's name.
        :rtype: tuple[str, str]
        """
        sensor_name = sensor.attrib["name"]

        # Find the path to the sensor's parent link --> must be a rigid body and have the sensor's name
        predicate = lambda path: get_rigid_body_enabled(path) and path.endswith(
            sensor_name
        )
        sensor_prim_path = isaacsim.core.utils.prims.get_prim_path(
            isaacsim.core.utils.prims.get_first_matching_child_prim(
                robot_prim_path, predicate
            )
        )

        # Define the sensor path with the path to the sensor's parent link, the sensor name and the sensor type
        sensor_prim_path = sensor_prim_path + f"/{sensor_name}_{sensor_type}"

        return sensor_prim_path, sensor_name

    @staticmethod
    def create_sensor_prim(
        sensor_type: str,
        sensor_path: str,
        config_name: Optional[str] = None,
        config_file_path: Optional[str] = None,
        relative_sensor_prim_path: Optional[str] = None,
        resolution: Optional[list[int]] = [1280, 800],
        clipping_range: Optional[list[int]] = [0.5, 10000000.0],
    ) -> Lidar | Camera:
        """Creates an RTX LiDAR or camera sensor prim at the given path in the Isaac Sim stage.

        :param sensor_type: Type of the sensor to create. Can be "lidar", "rgb_camera" or "depth_camera".
        :type sensor_type: str
        :param sensor_path: Path at which to create the sensor.
        :type sensor_path: str
        :param config_name: Name of the LiDAR config.
        :type config_name: str
        :param config_file_path: Path to the LiDAR or camera config file. Can be a USD file or a JSON file.
        :type config_file_path: str
        :param relative_sensor_prim_path: Relative path to the sensor prim in the USD file. Only used if the config
            file is a USD file. If it is None, the sensor prim will be created at the given sensor_path.
        :type relative_sensor_prim_path: str
        :param resolution: Resolution of the camera sensor. Defaults to [1280, 800].
        :type resolution: list[int], optional
        :param clipping_range: Clipping range of the camera sensor. Defaults to [0.5, 10000000.0].
        :return: The sensor prim.
        :rtype: Usd.Prim
        """
        if sensor_type == "lidar":
            # If the sensor is given as a USD file, create the sensor prim at the given path
            if config_file_path is not None and config_file_path.endswith(".usd"):
                isaacsim.core.utils.prims.create_prim(
                    sensor_path, "Xform", usd_path=config_file_path
                )

            # Get the path to the actual sensor prim in the stage (might not be the prim path, if a USD file is used)
            if relative_sensor_prim_path is None:
                lidar_path = sensor_path
            else:
                lidar_path = sensor_path + "/" + relative_sensor_prim_path

            # Create and initialize the LiDAR sensor
            sensor_prim = Lidar(
                lidar_config_name=config_name,
                prim_path=lidar_path,
                name=lidar_path.split("/")[-1],
            )
            sensor_prim.initialize()
        elif sensor_type in ["rgb_camera", "depth_camera"]:
            if config_file_path.endswith(".usd"):
                # Get path to the actual sensor prim in the stage (might not be the prim path, if a USD file is used)
                if relative_sensor_prim_path is None:
                    camera_path = sensor_path
                else:
                    camera_path = sensor_path + "/" + relative_sensor_prim_path

                # Create and initialize the camera sensor from a USD file
                isaacsim.core.utils.prims.create_prim(
                    sensor_path, "Xform", usd_path=config_file_path
                )
                sensor_prim = Camera(
                    prim_path=camera_path,
                    name=sensor_path.split("/")[-1],
                    resolution=resolution,
                )
                sensor_prim.initialize()
                sensor_prim.set_clipping_range(
                    near_distance=clipping_range[0], far_distance=clipping_range[1]
                )
            else:
                # Create and initialize the camera with properties from the given config file
                with open(config_file_path, "r") as file:
                    config_data = json.load(file)
                sensor_prim = Camera(
                    prim_path=sensor_path,
                    name=sensor_path.split("/")[-1],
                    frequency=config_data["frequency"],
                    dt=config_data["dt"],
                    resolution=config_data["resolution"],
                )
                sensor_prim.initialize()
                if config_data["focal_length"] is not None:
                    sensor_prim.set_focal_length(config_data["focal_length"])
                if config_data["focus_distance"] is not None:
                    sensor_prim.set_focus_distance(config_data["focus_distance"])
                if config_data["f_stop"] is not None:
                    sensor_prim.set_lens_aperture(config_data["f_stop"])
                if config_data["horizontal_aperture"] is not None:
                    sensor_prim.set_horizontal_aperture(
                        config_data["horizontal_aperture"]
                    )
                if config_data["vertical_aperture"] is not None:
                    sensor_prim.set_vertical_aperture(config_data["vertical_aperture"])
                if config_data["clipping_range"] is not None:
                    sensor_prim.set_clipping_range(
                        near_distance=config_data["clipping_range"][0],
                        far_distance=config_data["clipping_range"][1],
                    )
                if config_data["projection_type"] is not None:
                    sensor_prim.set_projection_type(config_data["projection_type"])
                if config_data["projection_mode"] is not None:
                    sensor_prim.set_projection_mode(config_data["projection_mode"])
                if config_data["stereo_role"] is not None:
                    sensor_prim.set_stereo_role("mono")
                if config_data["shutter_properties"] is not None:
                    sensor_prim.set_shutter_properties(
                        delay_open=config_data["shutter_properties"][0],
                        delay_close=config_data["shutter_properties"][1],
                    )
        return sensor_prim

    @staticmethod
    def add_lidar_config_path(config_dirname: str) -> None:
        """Add the path of the LiDAR config to the lidar profile base folder so that it can be found when creating a
         LiDAR sensor in Isaac Sim.

        :param config_dirname: Parent directory of the config file.
        :type config_dirname: str
        """
        carb_settings = carb.settings.get_settings()
        lidar_config_folders = carb_settings.get(
            "/app/sensors/nv/lidar/profileBaseFolder"
        )
        lidar_config_folders.append(config_dirname)
        carb_settings.set(
            "/app/sensors/nv/lidar/profileBaseFolder", lidar_config_folders
        )

    def create_sensors_from_urdf(
        self, robot_prim_path: str, urdf_path: Optional[str] = None
    ) -> None:
        """Create sensors that are defined in the given URDF file.

        :param robot_prim_path: Path to the robot prim in the stage.
        :type robot_prim_path: str
        :param urdf_path: Path to the URDF file. If it is None, the urdf_path instance variable is used. Defaults to
         None.
        :type urdf_path: Optional[str], optional
        """
        self.urdf_path = urdf_path or self.urdf_path

        urdf_root = ET.parse(self.urdf_path).getroot()
        # Iterate through all sensor elements
        for sensor in urdf_root.findall("./isaac/sensor"):
            sensor_type = sensor.attrib["type"]  # lidar, rgb_camera, depth_camera
            if not sensor_type in ["lidar", "rgb_camera", "depth_camera"]:
                carb.log_warn(
                    f"Sensor type '{sensor_type}' currently not supported to import from URDF. File '{self.urdf_path}'."
                )
                continue

            # Define all required names and paths for the sensor
            config = sensor.find("./config")
            urdf_dir = os.path.dirname(self.urdf_path)
            config_name = None
            config_file_path = None
            relative_sensor_prim_path = None

            if "file" in config.attrib.keys():
                config_name = config.attrib["file"]

                if (
                    "sensor" in config.attrib.keys()
                    and config_name.endswith(".usd")
                    and config_name.startswith("http")
                ):
                    relative_sensor_prim_path = config.attrib["sensor"]

                config_file = config_name
                config_name, config_file_path = (
                    ProximityURDFImporter.get_config_name_and_path(
                        config_file=config_name, urdf_dir=urdf_dir
                    )
                )

                if sensor_type == "lidar" and config_file.endswith(".json"):
                    config_dirname = os.path.dirname(config_file_path)
                    ProximityURDFImporter.add_lidar_config_path(
                        config_dirname=config_dirname
                    )
            else:
                config_name = config.attrib["name"]

            # Get sensor prim path
            sensor_prim_path, sensor_name = ProximityURDFImporter.get_sensor_prim_path(
                sensor=sensor, robot_prim_path=robot_prim_path, sensor_type=sensor_type
            )

            # Create the sensor
            sensor_prim = ProximityURDFImporter.create_sensor_prim(
                sensor_type=sensor_type,
                sensor_path=sensor_prim_path,
                config_name=config_name,
                config_file_path=config_file_path,
                relative_sensor_prim_path=relative_sensor_prim_path,
            )

            # Create the publishers
            frame_id = sensor.find("./frame_id")
            if frame_id is not None:
                frame_id = frame_id.text
            else:
                frame_id = sensor_name

            topics = sensor.findall("topic")
            for t in topics:
                topic_name = t.attrib["name"]

                msg_type = None
                if "msg_type" in t.attrib.keys():
                    msg_type = t.attrib["msg_type"]

                if sensor_type == "lidar":
                    ProximityURDFImporter.attach_lidar_publisher(
                        lidar_sensor=sensor_prim,
                        frame_id=frame_id,
                        topic_name=topic_name,
                        msg_type=msg_type,
                        draw_points=self.draw_point_cloud,
                    )
                elif sensor_type in ["rgb_camera", "depth_camera"]:
                    ProximityURDFImporter.attach_camera_publisher(
                        camera_prim=sensor_prim,
                        topic_name=topic_name,
                        msg_type=msg_type,
                        frame_id=frame_id,
                    )

    @staticmethod
    def get_config_name_and_path(config_file: str, urdf_dir: str) -> tuple[str, str]:
        """Get the config name and config path from the config file attribute in the URDF file.

        :param config_file: Config file attribute as given in the URDF file.
        :type config_file: str
        :param urdf_dir: Parent directory of the URDF file.
        :type urdf_dir: str
        :return: The config name and config file path.
        :rtype: tuple[str, str]
        """
        config_name = config_file
        config_file_path = None

        # The JSON or USD config file is given as a path relative to the URDF file
        # Or if the config is just given by its name, no path is required
        if config_name.endswith(".json") or config_name.endswith(".usd"):
            if not config_name.startswith("http"):
                config_file_path = os.path.abspath(os.path.join(urdf_dir, config_name))
            else:
                config_file_path = config_name
            _, config_filename = os.path.split(config_file_path)

            config_name = os.path.splitext(config_filename)[0]

        return config_name, config_file_path

    @staticmethod
    def attach_lidar_publisher(
        lidar_sensor: Lidar,
        frame_id: str,
        topic_name: str,
        draw_points: bool = False,
        msg_type: Optional[str] = None,
        qos_profile: Optional[str] = None,
    ) -> None:
        """Attaches a ROS 2 LaserScan or PointCloud2 publisher to the given LiDAR sensor prim.

        :param lidar_sensor: The LiDAR sensor prim.
        :type lidar_sensor: Usd.Prim
        :param frame_id: The frame ID.
        :type frame_id: str
        :param topic_name: Topic name on which to publish the data.
        :type topic_name: str
        :param draw_points: Whether to draw the point cloud in the app viewport. Defaults to False.
        :type draw_points: bool
        :param msg_type: The type of message. Can be "LaserScan" or "PointCloud2". If None, no publisher is created.
        :type msg_type: Optional[str]
        :param qos_profile: QoS Profile as a JSON string. Set to "{"createProfile": "Custom",
         "history": "keepLast", "depth": 10, "reliability": "bestEffort", "durability": "volatile", "deadline": 0.0,
         "lifespan": 0.0, "liveliness": "systemDefault", "leaseDuration": 0.0}" if None. Defaults to None.
        :type qos_profile: Optional[str]
        """
        if qos_profile is None:
            qos_profile = json.dumps(
                {
                    "createProfile": "Custom",
                    "history": "keepLast",
                    "depth": 10,
                    "reliability": "bestEffort",
                    "durability": "volatile",
                    "deadline": 0.0,
                    "lifespan": 0.0,
                    "liveliness": "systemDefault",
                    "leaseDuration": 0.0,
                }
            )

        if msg_type is not None:
            if "LaserScan" in msg_type:
                lidar_sensor.attach_laser_scan_publisher(
                    topic_name=topic_name, frame_id=frame_id, qos_settings=qos_profile
                )
            elif "PointCloud2" in msg_type:
                lidar_sensor.attach_point_cloud_publisher(
                    topic_name=topic_name, frame_id=frame_id, qos_settings=qos_profile
                )
            else:
                carb.log_warn(
                    f"Message type '{msg_type}' currently not supported for LiDAR sensor. Using sensor_msgs/msg/PointCloud2"
                )

        if draw_points:
            lidar_sensor.draw_point_cloud()

    @staticmethod
    def attach_camera_publisher(
        camera_prim: Camera,
        frame_id: str,
        topic_name: str,
        msg_type: str,
        queue_size: int = 1,
    ) -> None:
        """Attaches a ROS 2 CameraInfo, Image, or PointCloud2 publisher to the given camera prim.

        :param camera_prim: The camera prim.
        :type camera_prim: Camera
        :param frame_id: The frame id.
        :type frame_id: str
        :param topic_name: The topic name.
        :type topic_name: str
        :param msg_type: The type of message. It can be "CameraInfo", "Image", or "PointCloud2".
        :type msg_type: str
        :param queue_size: The queue size. Defaults to 1.
        :type queue_size: int, optional
        """
        if "CameraInfo" in msg_type:
            camera_prim.attach_camera_info_publisher(
                topic_name=topic_name, frame_id=frame_id, queue_size=queue_size
            )
        elif "Image" in msg_type or "PointCloud2" in msg_type:
            if "Image" in msg_type:
                camera_prim.attach_image_publisher(
                    depth="depth_camera" in camera_prim.name,
                    topic_name=topic_name,
                    frame_id=frame_id,
                    queue_size=queue_size,
                )
            elif "PointCloud2" in msg_type:
                camera_prim.attach_point_cloud_publisher(
                    topic_name=topic_name,
                    frame_id=frame_id,
                    queue_size=queue_size,
                )
