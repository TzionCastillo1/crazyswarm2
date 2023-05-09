from __future__ import annotations

from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from ..sim_data_types import State, Action
import socket
import select
import numpy as np

class Backend:
    """Backend that uses Gazebo physics simulation"""

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.t = 0
        #self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        node.create_subscription(Clock, '/clock', self._clock_cb, 10)
        

    def time(self) -> float:
        return self.t

    def shutdown(self):
        pass   

    def _clock_cb(self, msg):
        self.t = float(msg.clock.sec) + float(msg.clock.nanosec)/1e9
        #self.t = msg.clock.nanosec / 1e9

