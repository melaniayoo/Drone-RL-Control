import rclpy
from gymnasium import Env
import numpy as np
import gymnasium.spaces import Dict, Box
import math
from drone_control import DroneController

class DroneEnv(DroneController, Env):

    def __init__(self):
        super().__init__()

        self.get_logger().info("Starting all drone controllers...")

        