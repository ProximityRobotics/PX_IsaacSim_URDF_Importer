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
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

class SimulationTimePublisher(Node):
    def __init__(self, world):
        super().__init__("simulation_time_publisher")

        self.world = world
        self.clock_pub = self.create_publisher(Clock, "clock", 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        seconds_float = self.world.current_time
        seconds = int(seconds_float)
        nanoseconds = int((seconds_float - seconds) * 1e9)

        time_stamp = Time()
        time_stamp.sec = seconds
        time_stamp.nanosec = nanoseconds

        clock_msg = Clock()
        clock_msg.clock = time_stamp

        self.clock_pub.publish(clock_msg)
        