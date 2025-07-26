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

import json
import argparse


# Parse arguments
parser = argparse.ArgumentParser(
    description="The simulation example script loads a scene and imports a robot.",
    add_help=False,
)

# Manually add --help to avoid script exiting before SimulationApp help info message can be shown as well
parser.add_argument(
    "-h", "--help", action="store_true", help="Show this help message.", default=False
)
parser.add_argument(
    "--fix-base", help="Fix the robot's base link.", action="store_true", default=False
)
parser.add_argument(
    "--xacro-mappings",
    type=json.loads,
    help='Mappings for the xacro robot description file in JSON format, e.g., \'\{"key":"value"\}\'.',
    default=None,
)
parser.add_argument(
    "--joint-commands-topic",
    type=str,
    help="The name for the joint commands topic.",
    default="/joint_commands",
)
parser.add_argument(
    "--joint-states-topic",
    type=str,
    help="The name for the joint states topic.",
    default="/joint_states",
)
parser.add_argument(
    "--viewport-dimensions",
    type=int,
    nargs=2,
    metavar=("WIDTH", "HEIGHT"),
    help="The width and height of the Isaac Sim viewport, e.g., 1280 720.",
    default=[1280, 720],
)
parser.add_argument(
    "--window-dimensions",
    type=int,
    nargs=2,
    metavar=("WIDTH", "HEIGHT"),
    help="The width and height of the Isaac Sim window, e.g., 1920 1080.",
    default=[1920, 1080],
)
parser.add_argument(
    "--robot-position",
    type=float,
    nargs=3,
    metavar=("X", "Y", "Z"),
    help="The position of the robot in the world.",
    default=[0.0, 0.0, 0.0],
)
parser.add_argument(
    "--robot-orientation",
    type=float,
    nargs=4,
    metavar=("W", "X", "Y", "Z"),
    help="The orientation of the robot in the world.",
    default=[1.0, 0.0, 0.0, 0.0],
)
parser.add_argument(
    "--scene-position",
    type=float,
    nargs=3,
    metavar=("X", "Y", "Z"),
    help="The position of the scene in the world.",
    default=[0.0, 0.0, 0.0],
)
parser.add_argument(
    "--scene-orientation",
    type=float,
    nargs=4,
    metavar=("W", "X", "Y", "Z"),
    help="The orientation of the scene in the world.",
    default=[1.0, 0.0, 0.0, 0.0],
)
parser.add_argument(
    "--scene-scale",
    type=float,
    nargs=3,
    metavar=("X", "Y", "Z"),
    help="The scaling factor of the scene in x, y, and z direction.",
    default=[1.0, 1.0, 1.0],
)
parser.add_argument(
    "--draw-point-cloud",
    help="Draw the point cloud of any LiDAR imported from the robot description file.",
    action="store_true",
    default=False,
)
parser.add_argument(
    "--enable-gpu-dynamics",
    help="Enable GPU dynamics. Required for particle systems.",
    action="store_true",
    default=False,
)

args = parser.parse_args()

if args.help:
    parser.print_help()
    print("\n\n")


# Start the simulation app
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "width": args.viewport_dimensions[0],
        "height": args.viewport_dimensions[1],
        "window_width": args.window_dimensions[0],
        "window_height": args.window_dimensions[1],
        "headless": True,
        "renderer": "RaytracedLighting",
        "hide_ui": False,
    },
)

# Imports
import os
from typing import Optional

import rclpy

import carb
import isaacsim.core
import omni.kit.app

# Enable ros2 bridge
isaacsim.core.utils.extensions.enable_extension("isaacsim.ros2.bridge")

# More imports after ros2 bridge is enabled
from proximity_urdf_importer import ProximityURDFImporter
from ros2_nodes.articulation_controller import ArticulationController
from ros2_nodes.simulation_time_pub import SimulationTimePublisher


def set_livestream_settings() -> None:
    """Enables WebRTC livestreaming and disables mouse drawing in the viewport."""
    app.set_setting("/app/window/drawMouse", False)
    isaacsim.core.utils.extensions.enable_extension("omni.kit.livestream.webrtc")


def setup_environment(
    world: isaacsim.core.api.World,
    fix_base: bool,
    xacro_mappings: Optional[dict] = None,
) -> isaacsim.core.prims.Articulation:
    """Creates the environment by loading a scene and importing a robot.

    :param world: The world object.
    :type world: isaacsim.core.api.World
    :param fix_base: Whether to fix the robot's base link.
    :type fix_base: bool
    :param xacro_mappings: Mappings for processing the xacro file, if there is an xacro file. Defaults to None.
    :type xacro_mappings: Optional[dict], optional
    :return: The imported robot articulation.
    :rtype: isaacsim.core.prims.Articulation
    """
    scene_description_file = os.environ["SCENE_DESCRIPTION"]
    robot_description_file = os.environ["ROBOT_DESCRIPTION"]

    # Create scene
    if scene_description_file:
        # Load the given USD scene
        if not scene_description_file.startswith(
            "http"
        ) and not scene_description_file.startswith("/"):
            # If the given scene description file is a relative path, prepend the path to the data folder
            scene_description_file = "/isaac-sim/project/data/" + scene_description_file
        isaacsim.core.utils.prims.create_prim(
            prim_path="/World/Scene",
            prim_type="Xform",
            usd_path=scene_description_file,
            position=args.scene_position,
            orientation=args.scene_orientation,
            scale=args.scene_scale,
        )
    else:
        # If no USD scene is given, create an empty scene with just a ground plane and a dome light
        world.scene.add(
            isaacsim.core.api.objects.ground_plane.GroundPlane(
                prim_path="/World/Scene/GroundPlane", name="ground", z_position=0.0
            )
        )
        isaacsim.core.utils.prims.create_prim(
            prim_path="/World/Scene/DomeLight",
            prim_type="DomeLight",
            attributes={"inputs:intensity": 1000.0},
            orientation=isaacsim.core.utils.rotations.euler_angles_to_quat(
                (0, 45, 90), degrees=True
            ),
        )

    # Import robot
    if robot_description_file:
        if not robot_description_file.startswith("/"):
            # If the given robot description file is a relative path, prepend the path to the data folder
            robot_description_file = "/isaac-sim/project/data/" + robot_description_file

        importer = ProximityURDFImporter(
            simulation_app=app, draw_point_cloud=args.draw_point_cloud
        )

        if robot_description_file.endswith(".xacro"):
            robot, _ = importer.import_robot_from_xacro(
                xacro_path=robot_description_file,
                urdf_path=robot_description_file.replace(".xacro", ".urdf"),
                usd_path=robot_description_file.replace(".xacro", ".usd"),
                position=args.robot_position,
                orientation=args.robot_orientation,
                fix_base=fix_base,
                xacro_mappings=xacro_mappings,
            )
        elif robot_description_file.endswith(".urdf"):
            robot, _ = importer.import_robot_from_urdf(
                urdf_path=robot_description_file,
                usd_path=robot_description_file.replace(".urdf", ".usd"),
                position=args.robot_position,
                orientation=args.robot_orientation,
                fix_base=fix_base,
            )
        else:
            carb.log_error(
                f"The file {robot_description_file} is not a valid robot description file. "
                + "Not importing any robot."
            )
            return None
        return robot
    return None


def init_nodes(
    world: isaacsim.core.api.World,
    articulation: Optional[isaacsim.core.prims.Articulation],
) -> list[rclpy.node.Node]:
    """Initializes the ros2 nodes.

    :param world: The world object.
    :type world: isaacsim.core.api.World
    :param articulation: The robot articulation.
    :type articulation: isaacsim.core.prims.Articulation
    :return: A list of all ros2 nodes.
    :rtype: list[rclpy.node.Node]
    """
    # Publish simunlation time (Clock messages)
    clock = SimulationTimePublisher(world=world)

    # Publish robot joint states and receive robot joint commands (JointState messages)
    if articulation:
        joint_commands_topic_name = (
            articulation.joint_commands_topic_name or args.joint_commands_topic
        )
        joint_states_topic_name = (
            articulation.joint_states_topic_name or args.joint_states_topic
        )
        articulation_controller = ArticulationController(
            articulation=articulation,
            world=world,
            joint_commands_topic_name=joint_commands_topic_name,
            joint_states_topic_name=joint_states_topic_name,
        )
        return [clock, articulation_controller]
    else:
        return [clock]


def run_simulation(
    world: isaacsim.core.api.World,
    articulation: Optional[isaacsim.core.prims.Articulation],
) -> None:
    """Runs the simulation.

    :param world: The world object.
    :type world: isaacsim.core.api.World
    :param articulation: The robot articulation.
    :type articulation: isaacsim.core.prims.Articulation
    """
    rclpy.init()

    nodes = init_nodes(world, articulation)

    step_counter = 0

    world.reset()
    while app.is_running():
        world.step()

        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.01)

        step_counter += 1
        if world.is_playing():
            if step_counter == 1 and articulation:
                articulation.initialize()
        else:
            step_counter = 0

    for node in nodes:
        node.destroy_node()


def set_particle_display_settings(
    particle_viz_mode: str = "all",
    display_particles: bool = False,
    display_fluid_surface: bool = False,
    display_cloth_particles: bool = False,
    display_cloth_mesh: bool = False,
) -> None:
    """Sets the particle display settings.

    :param particle_viz_mode: The particle visualization mode. Can be "all", "selected", or "none". Defaults to "all".
    :type particle_viz_mode: str, optional
    :param display_particles: Whether to show particles in the viewport. Defaults to False.
    :type display_particles: bool, optional
    :param display_fluid_surface: Whether to show fluid surface particles in the viewport. Defaults to False.
    :type display_fluid_surface: bool, optional
    :param display_cloth_particles: Whether to show cloth particles in the viewport. Defaults to False.
    :type display_cloth_particles: bool, optional
    :param display_cloth_mesh: Whether to show cloth mesh in the viewport. Defaults to False.
    :type display_cloth_mesh: bool, optional
    """
    settings_interface = carb.settings.acquire_settings_interface()

    if particle_viz_mode == "all":
        settings_interface.set(
            omni.physx.bindings._physx.SETTING_DISPLAY_PARTICLES,
            omni.physx.bindings._physx.VisualizerMode.ALL,
        )
    elif particle_viz_mode == "selected":
        settings_interface.set(
            omni.physx.bindings._physx.SETTING_DISPLAY_PARTICLES,
            omni.physx.bindings._physx.VisualizerMode.SELECTED,
        )
    elif particle_viz_mode == "none":
        settings_interface.set(
            omni.physx.bindings._physx.SETTING_DISPLAY_PARTICLES,
            omni.physx.bindings._physx.VisualizerMode.NONE,
        )

    settings_interface.set(
        omni.physx.bindings._physx.SETTING_DISPLAY_PARTICLES_SHOW_PARTICLE_SET_PARTICLES,
        display_particles,
    )
    settings_interface.set(
        omni.physx.bindings._physx.SETTING_DISPLAY_PARTICLES_SHOW_FLUID_SURFACE,
        display_fluid_surface,
    )
    settings_interface.set(
        omni.physx.bindings._physx.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_PARTICLES,
        display_cloth_particles,
    )
    settings_interface.set(
        omni.physx.bindings._physx.SETTING_DISPLAY_PARTICLES_SHOW_CLOTH_MESH,
        display_cloth_mesh,
    )


def main():
    """Main method. Sets particle display settings, livestream settings, creates the world object, sets GPU dynamics
    settings, creates the environment, and runs the simulation.
    """
    if args.enable_gpu_dynamics:
        set_particle_display_settings()
    set_livestream_settings()

    world = isaacsim.core.api.World(
        stage_units_in_meters=1.0,
        physics_prim_path="/World/PhysicsScene",
    )
    if args.enable_gpu_dynamics:
        physics_context = world.get_physics_context()
        physics_context.set_broadphase_type("GPU")
        physics_context.enable_gpu_dynamics(flag=True)
        physics_context.enable_fabric(True)

    articulation = setup_environment(
        world, fix_base=args.fix_base, xacro_mappings=args.xacro_mappings
    )
    run_simulation(world, articulation)
    app.close()


if __name__ == "__main__":
    main()
