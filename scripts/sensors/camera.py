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
import json
import math

import omni.replicator.core as rep
import omni.graph.core as og
import omni.syntheticdata._syntheticdata as sd
from isaacsim.sensors.camera import Camera as CameraIsaacSim
from omni.syntheticdata import SyntheticData
from isaacsim.ros2.bridge import read_camera_info


class Camera(CameraIsaacSim):
    def __init__(
        self,
        prim_path: str = "/World/Sensors/Camera",
        name: str = "camera_sensor",
        resolution: list[int, int] = [1280, 720],
        frequency: Optional[float] = None,
        dt: Optional[float] = None,
    ):
        """Handles camera sensors.

        :param prim_path: Path to the camera sensor prim in the stage. Defaults to "/World/Sensors/Camera".
        :type prim_path: str, optional
        :param name: Unique name for the camera prim. Defaults to "camera_sensor".
        :type name: str, optional
        :param resolution: The camera's resolution in pixels. Defaults to [1280, 720].
        :type resolution: list[int, int], optional
        :param frequency: The camera sensor's frequency, i.e. how frequently the data frame is updated, in Hz. If it is
         None, the default of the Isaac Sim Camera ist used. Currently this default is 60.0. Defaults to None.
        :type frequency: Optional[float], optional
        :param dt: The period at which the data frame is updated in seconds. You can either set the frequency or the dt.
         If dt is None, the default of the Isaac Sim Camera ist used. Currently this default is 1/60.0. Defaults to
         None.
        :type dt: Optional[float], optional
        """
        super().__init__(
            prim_path=prim_path,
            name=name,
            resolution=resolution,
            frequency=frequency,
            dt=dt,
        )

    # @override
    def get_focal_length(self) -> float:
        """Returns the focal length in mm (if the default stage unit of 1 m is used). Overrides the Isaac Sim Camera
         class' function to avoid scaling.

        :return: The focal length.
        :rtype: float
        """
        return self.prim.GetAttribute("focalLength").Get()

    # @override
    def set_focal_length(self, value: float):
        """Sets the focal length in mm (if the default stage unit of 1 m is used). Overrides the Isaac Sim Camera class'
         function to avoid scaling.

        :param value: The desired focal length in mm (or 1/1000 stage units).
        :type value: float
        """
        self.prim.GetAttribute("focalLength").Set(value)

    # @override
    def get_horizontal_aperture(self) -> float:
        """Returns the horizontal aperture, i.e. the width of the sensor's active area, in mm (if the default stage unit
         of 1 m is used). Overrides the Isaac Sim Camera class' function to avoid scaling.

        :return: The horizontal aperture.
        :rtype: float
        """
        return self.prim.GetAttribute("horizontalAperture").Get()

    # @override
    def set_horizontal_aperture(self, value: float):
        """Sets the horizontal aperture in mm (if the default stage unit of 1 m is used). Overrides the Isaac Sim Camera
         class' function to avoid scaling.

        :param value: The desired horizontal aperture in mm (or 1/1000 stage units).
        :type value: float
        """
        self.prim.GetAttribute("horizontalAperture").Set(value)

    # @override
    def get_vertical_aperture(self) -> float:
        """Returns the vertical aperture, i.e. the height of the sensor's active area, in mm (if the default stage unit
         of 1 m is used). Overrides the Isaac Sim Camera class' function to avoid scaling.

        :return: The vertical aperture.
        :rtype: float
        """
        return self.prim.GetAttribute("verticalAperture").Get()

    # @override
    def set_vertical_aperture(self, value: float):
        """Sets the vertical aperture in mm (if the default stage unit of 1 m is used). Overrides the Isaac Sim Camera
         class' function to avoid scaling.

        :param value: The desired vertical aperture in mm (or 1/1000 stage units).
        :type value: float
        """
        self.prim.GetAttribute("verticalAperture").Set(value)

    # @override
    def get_horizontal_fov(self) -> float:
        """Calculates the horizontal field of view (FOV) in degrees. Overrides the Isaac Sim Camera class' function to 
         get the value in degrees as it is used more frequently when describing a camera's FOV.

        :return: The horizontal field of view in degrees.
        :rtype: float
        """
        return math.degrees(
            2
            * math.atan(self.get_horizontal_aperture() / (2 * self.get_focal_length()))
        )

    # @override
    def get_vertical_fov(self) -> float:
        """Calculates the vertical field of view (FOV) in degrees. Overrides the Isaac Sim Camera class' function to 
         calculate it independently from the horizontal FOV and resolution and to get the value in degrees as it is
         used more frequently when describing a camera's FOV.

        :return: The vertical field of view in degrees.
        :rtype: float
        """
        return math.degrees(
            2 * math.atan(self.get_vertical_aperture() / (2 * self.get_focal_length()))
        )

    def attach_camera_info_publisher(
        self,
        topic_name: str = "camera_info",
        frame_id: str = "camera_sensor",
        qos_settings: Optional[str] = None,
        node_namespace: str = "",
        queue_size: int = 1,
    ) -> None:
        """Attaches a ROS 2 CameraInfo publisher to the camera prim.

        :param topic_name: The topic name. Defaults to "camera_info".
        :type topic_name: str, optional
        :param frame_id: The frame ID. Defaults to "camera_sensor".
        :type frame_id: str, optional
        :param qos_settings: QoS Profile as a JSON string. Set to "{"createProfile": "Custom", "history": "keepLast",
         "depth": 10, "reliability": "reliable", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0,
         "liveliness": "systemDefault", "leaseDuration": 0.0}" if None. Defaults to None.
        :type qos_settings: Optional[str], optional
        :param node_namespace: The node namespace. Defaults to "".
        :type node_namespace: str, optional
        :param queue_size: The queue size. Defaults to 1.
        :type queue_size: int, optional
        """
        writer = rep.writers.get("ROS2PublishCameraInfo")
        step_size = int(60 / self.get_frequency())

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

        camera_info = read_camera_info(render_product_path=self._render_product_path)
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name,
            width=self.get_resolution()[0],
            height=self.get_resolution()[1],
            projectionType=self.get_projection_type(),
            k=camera_info["k"].reshape([1, 9]),
            r=camera_info["r"].reshape([1, 9]),
            p=camera_info["p"].reshape([1, 12]),
            physicalDistortionModel=camera_info["physicalDistortionModel"],
            physicalDistortionCoefficients=camera_info[
                "physicalDistortionCoefficients"
            ],
            qosProfile=qos_settings,
        )
        writer.attach([self._render_product_path])
        gate_path = SyntheticData._get_node_path(
            "PostProcessDispatch" + "IsaacSimulationGate", self._render_product_path
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    def attach_image_publisher(
        self,
        depth: bool = False,
        topic_name: str = "camera_info",
        frame_id: str = "camera_sensor",
        qos_settings: Optional[str] = None,
        node_namespace: str = "",
        queue_size: int = 1,
    ):
        """Attaches a ROS 2 Image publisher to the camera prim.

        :param depth: Whether it should be depth images. Else it will be RGB color images. Defaults to False.
        :type depth: bool, optional
        :param topic_name: The topic name. Defaults to "camera_info".
        :type topic_name: str, optional
        :param frame_id: The frame ID. Defaults to "camera_sensor".
        :type frame_id: str, optional
        :param qos_settings: QoS Profile as a JSON string. Set to "{"createProfile": "Custom", "history": "keepLast",
         "depth": 10, "reliability": "reliable", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0,
         "liveliness": "systemDefault", "leaseDuration": 0.0}" if None. Defaults to None.
        :type qos_settings: Optional[str], optional
        :param node_namespace: The node namespace. Defaults to "".
        :type node_namespace: str, optional
        :param queue_size: The queueu size. Defaults to 1.
        :type queue_size: int, optional
        """
        if depth:
            rv = SyntheticData.convert_sensor_type_to_rendervar(
                sd.SensorType.DistanceToImagePlane.name
            )
        else:
            rv = SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        step_size = int(60 / self.get_frequency())

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

        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name,
        )
        writer.attach([self._render_product_path])
        gate_path = SyntheticData._get_node_path(
            "PostProcessDispatch" + "IsaacSimulationGate", self._render_product_path
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    def attach_point_cloud_publisher(
        self,
        topic_name: str = "camera_info",
        frame_id: str = "camera_sensor",
        qos_settings: Optional[str] = None,
        node_namespace: str = "",
        queue_size: int = 1,
    ):
        """Attaches a ROS 2 PointCloud2 publisher to the camera prim.

        :param topic_name: The topic name. Defaults to "camera_info".
        :type topic_name: str, optional
        :param frame_id: The frame ID. Defaults to "camera_sensor".
        :type frame_id: str, optional
        :param qos_settings: QoS Profile as a JSON string. Set to "{"createProfile": "Custom", "history": "keepLast",
         "depth": 10, "reliability": "reliable", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0,
         "liveliness": "systemDefault", "leaseDuration": 0.0}" if None. Defaults to None.
        :type qos_settings: Optional[str], optional
        :param node_namespace: The node namespace. Defaults to "".
        :type node_namespace: str, optional
        :param queue_size: The queue size. Defaults to 1.
        :type queue_size: int, optional
        """
        rv = SyntheticData.convert_sensor_type_to_rendervar(
            sd.SensorType.DistanceToImagePlane.name
        )
        step_size = int(60 / self.get_frequency())

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

        writer = rep.writers.get(rv + "ROS2PublishPointCloud")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name,
        )
        writer.attach([self._render_product_path])
        gate_path = SyntheticData._get_node_path(
            "PostProcessDispatch" + "IsaacSimulationGate", self._render_product_path
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
