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
from isaacsim.core.prims import Articulation as IsaacArticulation


class Articulation(IsaacArticulation):
    def __init__(
        self,
        *args,
        joint_states_topic_name: Optional[str] = None,
        joint_commands_topic_name: Optional[str] = None,
        **kwargs
    ):
        super().__init__(*args, **kwargs)
        self.joint_states_topic_name = joint_states_topic_name
        self.joint_commands_topic_name = joint_commands_topic_name
