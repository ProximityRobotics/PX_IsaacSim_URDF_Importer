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

from isaacsim.core.utils.types import ArticulationActions


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
                        self.articulation.get_joint_positions()[0].tolist()
                    )
                    joint_state_msg.velocity = (
                        self.articulation.get_joint_velocities()[0].tolist()
                    )
                    joint_state_msg.effort = (
                        self.articulation.get_measured_joint_efforts()[0].tolist()
                    )

                    joint_state_msg.header.stamp = msg.clock

                    self.joint_states_pub.publish(joint_state_msg)
                except AttributeError:
                    pass  # articulation not initialized yet

    
    def joint_commands_callback(self, msg):
        if not self.world.is_playing():
            return
        
        try:
            position_joint_indices = []
            joint_positions_new = []
            joint_velocities_new = []
            joint_efforts_new = []

            for idx, name in enumerate(msg.name):
                dof_index = self.articulation.get_dof_index(name)
                position_joint_indices.append(dof_index)

                joint_positions_new.append(msg.position[idx] if msg.position else None)
                joint_velocities_new.append(msg.velocity[idx] if msg.velocity else None)
                joint_efforts_new.append(msg.effort[idx] if msg.effort else None)

            def none_if_all_none(lst):
                return None if all(x is None for x in lst) else lst
            joint_positions_new = none_if_all_none(joint_positions_new)
            joint_velocities_new = none_if_all_none(joint_velocities_new)
            joint_efforts_new = none_if_all_none(joint_efforts_new)
                
            action = ArticulationActions(
                joint_positions=joint_positions_new,
                joint_velocities=joint_velocities_new,
                joint_efforts=joint_efforts_new,
                joint_indices=position_joint_indices,
            )
            self.articulation.apply_action(action)
        except AttributeError:
            # Articulation not initialized yet --> 'ArticulationController' object has no attribute '_articulation_view'
            pass
