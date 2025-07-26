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

from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

from omni.isaac.core.utils.types import ArticulationAction


class ArticulationController(Node):
    def __init__(
        self,
        articulation,
        world,
        joint_commands_topic_name: str,
        joint_states_topic_name: str,
    ):
        super().__init__("articulation_controller")

        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.articulation = articulation
        self.world = world

        self.joint_commands_sub = self.create_subscription(
            JointState,
            joint_commands_topic_name,
            self.joint_commands_callback,
            10,
        )

        self.clock_sub = self.create_subscription(
            Clock, "clock", self.clock_callback, 10
        )

        self.joint_states_pub = self.create_publisher(
            JointState, joint_states_topic_name, 10
        )

    def clock_callback(self, msg):
        if self.world.is_playing():
            joint_names = self.articulation.dof_names

            if joint_names is not None:
                try:
                    joint_state_msg = JointState()
                    joint_state_msg.name = joint_names
                    joint_state_msg.position = (
                        self.articulation.get_joint_positions().tolist()
                    )
                    joint_state_msg.velocity = (
                        self.articulation.get_joint_velocities().tolist()
                    )
                    joint_state_msg.effort = (
                        self.articulation.get_measured_joint_efforts().tolist()
                    )

                    joint_state_msg.header.stamp = msg.clock

                    self.joint_states_pub.publish(joint_state_msg)
                except AttributeError:
                    pass  # articulation not initialized yet

    def joint_commands_callback(self, msg):
        joint_names = list(msg.name)
        joint_positions = list(msg.position).copy()
        joint_velocities = list(msg.velocity).copy()
        joint_efforts = list(msg.effort).copy()

        if self.world.is_playing():
            try:
                n_joints = len(joint_names)
                position_joint_indices = list()
                joint_positions_new = [None] * n_joints
                joint_velocities_new = [None] * n_joints
                joint_efforts_new = [None] * n_joints

                for idx, joint_name in enumerate(joint_names):
                    articulation_dof_idx = self.articulation.get_dof_index(joint_name)
                    position_joint_indices.append(articulation_dof_idx)
                    if joint_positions:
                        joint_positions_new[idx] = joint_positions[idx]
                    if joint_velocities:
                        joint_velocities_new[idx] = joint_velocities[idx]
                    if joint_efforts:
                        joint_efforts_new[idx] = joint_efforts[idx]

                action = ArticulationAction(
                    joint_positions=joint_positions_new,
                    joint_velocities=joint_velocities_new,
                    joint_efforts=joint_efforts_new,
                    joint_indices=position_joint_indices,
                )
                self.articulation.apply_action(action)
            except AttributeError:
                # Articulation not initialized yet --> 'ArticulationController' object has no attribute '_articulation_view'
                pass
