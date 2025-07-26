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

import omni.kit.commands
import omni.replicator.core as rep
from isaacsim.sensors.rtx import LidarRtx


class Lidar(LidarRtx):
    def __init__(
        self,
        lidar_config_name: str,
        prim_path: str = "/World/Sensors/Lidar",
        name: str = "lidar_sensor",
        position: Optional[list[float]] = None,
        translation: Optional[list[float]] = None,
        orientation: list[float] = [1.0, 0.0, 0.0, 0.0],
    ) -> None:
        """Handles RTX LiDAR sensors.

        :param lidar_config_name: Name of the LiDAR config JSON file without its extension. Can only be used if its
         directory is in the app's lidar profile base folder list or if it is located at
        data/config_files/lidar_configs.
        :type lidar_config_name: str
        :param prim_path: Path to the LiDAR prim in the stage. Defaults to "/World/Sensors/Lidar".
        :type prim_path: str, optional
        :param name: Unique name for the LiDAR sensor. Defaults to "lidar_sensor".
        :type name: str, optional
        :param position: The desired xyz position in the world coordinate system. If position and translation are both
         None, then the translation is used and set to [0.0, 0.0, 0.0]. Defaults to None.
        :param translation: The desired xyz translation relative to the parent prim. If position and translation are
         both None, then the translation is used and set to [0.0, 0.0, 0.0]. Defaults to None.
        :type translation: list[float], optional
        :param orientation: The desired orientation as quaternion wxyz. If position is used, the orientation is in the
         world coordinate system. If translation is used, the orientation is relative to the coordinate system of the
         parent prim. Defaults to [1.0, 0.0, 0.0, 0.0].
        :type orientation: list[float], optional
        """
        # TODO add lidar config name dir if not there yet
        self.lidar_config_name = lidar_config_name

        # Import here to avoid circular import
        from proximity_urdf_importer import ProximityURDFImporter

        ProximityURDFImporter.add_lidar_config_path(
            "/isaac-sim/project/data/config_files/lidar_configs"
        )

        if position is None and translation is None:
            translation = [0.0, 0.0, 0.0]

        # Create LiDAR sensor
        super().__init__(
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            config_file_name=lidar_config_name,
        )

        self.draw_lidar_writer = None

        self.lidar_scan_buffer = None

    def draw_point_cloud(self, buffer: bool = True) -> None:
        """Draws the LiDAR point cloud in the viewport.

        :param buffer: Whether to create a full scan buffer for the LiDAR point cloud data. Defaults to True.
        :type buffer: bool, optional
        """
        if buffer:
            self.draw_lidar_writer = rep.writers.get(
                "RtxLidarDebugDrawPointCloudBuffer"
            )
        else:
            self.draw_lidar_writer = rep.writers.get("RtxLidarDebugDrawPointCloud")
        self.draw_lidar_writer.initialize()
        self.draw_lidar_writer.attach([self._render_product_path])

    def stop_draw_point_cloud(self) -> None:
        """Stops drawing the LiDAR point cloud in the viewport."""
        if self.draw_lidar_writer:
            self.draw_lidar_writer.detach()
        self.draw_lidar_writer = None

    def attach_laser_scan_publisher(
        self,
        topic_name: str = "scan",
        frame_id: str = "lidar_sensor",
        qos_settings: Optional[str] = None,
    ) -> None:
        """Attaches a ROS 2 LaserScan publisher to the LiDAR sensor.

        :param topic_name: Name for the topic where theLaserScan messages will be published. Defaults to "scan".
        :type topic_name: str, optional
        :param frame_id: Name of the frame that serves as coordinate system for the LiDAR data. Defaults to
         "lidar_sensor".
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
        laser_scan_writer.initialize(
            topicName=topic_name, frameId=frame_id, qosProfile=qos_settings
        )
        laser_scan_writer.attach([self._render_product_path])

    def attach_point_cloud_publisher(
        self,
        topic_name: str = "points",
        frame_id: str = "lidar_sensor",
        buffer: bool = True,
        qos_settings: Optional[str] = None,
    ) -> None:
        """Attaches a ROS 2 PointCloud2 publisher to the LiDAR sensor.

        :param topic_name: Name for the topic where the PointCloud2 messages will be published. Defaults to "points".
        :type topic_name: str, optional
        :param frame_id: Name of the frame that serves as coordinate system for the LiDAR data. Defaults to
         "lidar_sensor".
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
        point_cloud_writer.attach([self._render_product_path])

    def get_lidar_scan_buffer_intensities(self) -> np.ndarray:
        """Returns the LiDAR LaserScan buffer intensities and initializes the LidarScan buffer if it does not exist yet.

        :return: The intensities.
        :rtype: np.ndarray
        """
        if self.lidar_scan_buffer is None:
            annotator = rep.AnnotatorRegistry.get_annotator(
                "RtxSensorCpuIsaacCreateRTXLidarScanBuffer"
            )
            annotator.initialize(outputTimestamp=True)
            annotator.attach(self._render_product_path)
            self.lidar_scan_buffer = annotator
        return self.lidar_scan_buffer.get_data()["intensity"]
