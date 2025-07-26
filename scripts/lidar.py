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

from typing import Optional
import numpy as np
import json

import omni.isaac.core
import omni.kit.commands
import omni.replicator.core as rep
from omni.isaac.sensor import LidarRtx
from pxr import Gf


# TODO inherit omni.isaac.sensor.LidarRtx as soon as lidar sensor created successfully with that class


class LiDAR:
    def __init__(
        self,
        lidar_config_name: str,
        prim_path: str = "/World/Sensors/LiDAR",
        translation: list[float] = [0.0, 0.0, 0.0],
        orientation: list[float] = [1.0, 0.0, 0.0, 0.0],
    ) -> None:
        """Handles RTX LiDAR sensors.

        :param lidar_config_name: Name of the lidar config JSON file. Can only be used if it is located at
         data/config_files/lidar_configs.
        :type lidar_config_name: str
        :param prim_path: Path to the lidar prim in the stage. Defaults to "/World/Sensors/LiDAR".
        :type prim_path: str, optional
        :param translation: The desired xyz translation relative to the parent prim. Defaults to [0.0, 0.0, 0.0].
        :type translation: list[float], optional
        :param orientation: The desired orientation relative to the parent prim as quaternion wxyz. Defaults to 
         [1.0, 0.0, 0.0, 0.0].
        :type orientation: list[float], optional
        """
        # Find a valid prim path
        self.lidar_config_name = lidar_config_name

        self.prim_path = omni.isaac.core.utils.string.find_unique_string_name(
            initial_name=prim_path,
            is_unique_fn=lambda x: not omni.isaac.core.utils.prims.is_prim_path_valid(
                x
            ),
        )

        # Create LiDAR sensor
        _, lidar_sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=self.prim_path,
            parent=None,
            config=self.lidar_config_name,
            translation=Gf.Vec3d(*translation),
            orientation=Gf.Quatd(*orientation),
        )  # lidar_sensor return type: pxr.Usd.Prim

        self.render_product = rep.create.render_product(
            lidar_sensor.GetPath().pathString, [1, 1]
        )  # camera, resolution

        self._lidar_rtx = LidarRtx(prim_path=self.prim_path)

        self.draw_lidar_writer = None

        self.lidar_scan_buffer = None

        # Enable ros2 bridge extension
        # Enabled by default, but somehow not everything is available unless enabled manually as well
        omni.isaac.core.utils.extensions.enable_extension("omni.isaac.ros2_bridge")

    def draw_point_cloud(self, buffer: bool = True) -> None:
        """Draws the LiDAR point cloud inside the viewport.

        :param buffer: Whether to create a full scan buffer for the LiDAR point cloud data. Defaults to True.
        :type buffer: bool, optional
        """
        if buffer:
            # TODO check whether RtxSensorCpuIsaacCreateRTXLidarScanBuffer required
            # annotator = rep.AnnotatorRegistry.get_annotator(
            #     "RtxSensorCpuIsaacCreateRTXLidarScanBuffer"
            # )
            # annotator.initialize(outputTimestamp=True)
            # annotator.attach(self.render_product)

            # self.lidar_scan_buffer = annotator
            # print(f"\n\n\n{annotator.get_data().keys()}\n\n\n")

            self.draw_lidar_writer = rep.writers.get(
                "RtxLidarDebugDrawPointCloudBuffer"
            )
        else:
            self.draw_lidar_writer = rep.writers.get("RtxLidarDebugDrawPointCloud")
        self.draw_lidar_writer.initialize()
        self.draw_lidar_writer.attach([self.render_product.path])

    def stop_draw_point_cloud(self) -> None:
        """Stops drawing the LiDAR point cloud."""
        if self.draw_lidar_writer:
            self.draw_lidar_writer.detach()
        self.draw_lidar_writer = None

    def attach_laser_scan_publisher(
        self,
        topic_name: str = "scan",
        frame_id: str = "lidar_sensor",
        qos_settings: Optional[str] = None,
    ) -> None:
        """Attaches a ros2 laser scan publisher (LaserScan messages) to the RTX LiDAR.

        :param topic_name: Name for the topic where the laser scan messages will be published. Defaults to "scan".
        :type topic_name: str, optional
        :param frame_id: Name of the frame that serves as coordinate system for the LiDAR data. Defaults to "lidar_sensor".
        :type frame_id: str, optional
        :param qos_settings: QoS Profile as a JSON string. Set to "{"createProfile": "Custom", "history": "keepLast",
         "depth": 10, "reliability": "reliable", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0,
         "liveliness": "systemDefault", "leaseDuration": 0.0}" if None. Defaults to None.
        :type qos_settings: Optional[str], optional
        """
        laser_scan_writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")

        if qos_settings is None:
            qos_settings = json.dumps(
                {
                    "createProfile": "Custom",
                    "history": "keepLast",
                    "depth": 10,
                    "reliability": "reliable",
                    "durability": "volatile",
                    "deadline": 0.0,
                    "lifespan": 0.0,
                    "liveliness": "systemDefault",
                    "leaseDuration": 0.0,
                }
            )

        # Available initialize parameters: frameId, nodeNamespace, queueSize, topicName, context, qosProfile
        laser_scan_writer.initialize(topicName=topic_name, frameId=frame_id, qosProfile=qos_settings)
        laser_scan_writer.attach([self.render_product])

        # TODO why warning and no scan topic?:
        # "IsaacComputeRTXLidarFlatScan only works with Rotary lidar, and robin_w_config is not one.""
        # RTX Lidar Nodes website: "It’s designed for use with 2D Lidar [...], but can be used with any rotary or solid state Lidar configuration."
        # --> maybe works with new isaac sim version

    def attach_point_cloud_publisher(
        self,
        topic_name: str = "points",
        frame_id: str = "lidar_sensor",
        buffer: bool = True,
        qos_settings: Optional[str] = None,
    ) -> None:
        """Attaches a ros2 point cloud publisher (PointCloud2 messages) to the RTX LiDAR.

        :param topic_name: Name for the topic where the point cloud messages will be published, defaults to "points"
        :type topic_name: str, optional
        :param frame_id: Name of the frame that serves as coordinate system for the LiDAR data. Defaults to "lidar_sensor".
        :type frame_id: str, optional
        :param buffer: Whether to create a full scan buffer for the LiDAR point cloud data. Defaults to True.
        :type buffer: bool, optional
        :param qos_settings: QoS Profile as a JSON string. Set to "{"createProfile": "Custom", "history": "keepLast",
         "depth": 10, "reliability": "reliable", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0,
         "liveliness": "systemDefault", "leaseDuration": 0.0}" if None. Defaults to None.
        :type qos_settings: Optional[str], optional
        """
        if buffer:
            point_cloud_writer = rep.writers.get(
                "RtxLidar" + "ROS2PublishPointCloudBuffer"
            )
        else:
            point_cloud_writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")

        if qos_settings is None:
            qos_settings = json.dumps(
                {
                    "createProfile": "Custom",
                    "history": "keepLast",
                    "depth": 10,
                    "reliability": "reliable",
                    "durability": "volatile",
                    "deadline": 0.0,
                    "lifespan": 0.0,
                    "liveliness": "systemDefault",
                    "leaseDuration": 0.0,
                }
            )

        # Available initialize parameters: frameId, nodeNamespace, queueSize, topicName, context, qosProfile
        point_cloud_writer.initialize(
            topicName=topic_name, frameId=frame_id, qosProfile=qos_settings
        )
        point_cloud_writer.attach([self.render_product])

    def initialize(self) -> None:
        """Creates a physics simulation view if it does not already exist and sets other properties using the PhysX
        tensor API.
        """
        self._lidar_rtx.initialize()

    def get_current_frame(self) -> dict[np.ndarray]:
        """Grants access to the current LiDAR frame. The current frame may contain multiple different data types
         depending on what has been "added" to the frame, e.g. via add_point_cloud_data_to_frame().

        :return: The current LiDAR frame.
        :rtype: dict[np.ndarray]
        """
        return self._lidar_rtx.get_current_frame()

    def get_lidar_scan_buffer_intensities(self) -> np.ndarray:
        if self.lidar_scan_buffer is None:
            annotator = rep.AnnotatorRegistry.get_annotator(
                "RtxSensorCpuIsaacCreateRTXLidarScanBuffer"
            )
            annotator.initialize(outputTimestamp=True)
            annotator.attach(self.render_product)
            self.lidar_scan_buffer = annotator
        return self.lidar_scan_buffer.get_data()["intensity"]

    def add_point_cloud_data_to_frame(self) -> None:
        """Adds point cloud data to the frame. Key: "point_cloud_data"."""
        self._lidar_rtx.add_point_cloud_data_to_frame()

    def remove_point_cloud_data_from_frame(self) -> None:
        """Removes point cloud data from the frame."""
        self._lidar_rtx.remove_point_cloud_data_from_frame()

    def add_linear_depth_data_to_frame(self) -> None:
        """Adds linear depth measurements to the frame."""
        self._lidar_rtx.add_linear_depth_data_to_frame()

    def remove_linear_depth_data_from_frame(self) -> None:
        """Removes linear depth measurements from the frame."""
        self._lidar_rtx.remove_linear_depth_data_from_frame()

    def add_intensities_data_to_frame(self) -> None:
        """Adds intensity measurements to the frame."""
        self._lidar_rtx.add_intensities_data_to_frame()

    def remove_intensities_data_from_frame(self) -> None:
        """Removes intensity measurements from the frame."""
        self._lidar_rtx.remove_intensities_data_from_frame()

    def add_range_data_to_frame(self) -> None:
        """Adds range data to the frame."""
        self._lidar_rtx.add_range_data_to_frame()

    def remove_range_data_from_frame(self) -> None:
        """Removes range data from the frame."""
        self._lidar_rtx.remove_range_data_from_frame()

    def add_azimuth_data_to_frame(self) -> None:
        """Adds azimuth data to the frame."""
        self._lidar_rtx.add_azimuth_data_to_frame()

    def remove_azimuth_data_from_frame(self) -> None:
        """Removes azimuth data from the frame."""
        self._lidar_rtx.remove_azimuth_data_from_frame()

    def add_elevation_data_to_frame(self) -> None:
        """Adds elevation data to the frame."""
        self._lidar_rtx.add_elevation_data_to_frame()

    def remove_elevation_data_from_frame(self) -> None:
        """Removes elevation data from the frame."""
        self._lidar_rtx.remove_elevation_data_from_frame()
