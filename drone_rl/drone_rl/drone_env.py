import rclpy
from gymnasium import Env
import numpy as np
from gymnasium.spaces import Dict, Box
import math
from drone_control import DroneController

class DroneEnv(DroneController, Env):

    def __init__(self):
        super().__init__()

        self.get_logger().info("Starting all drone controllers...")

        # TODO : Establish Training Parameters | DONE
        self.robot_name = "SJTU-Drone"
        self._target_location = np.array([1, 10], dtype=np.float32)
        self._initial_agent_location = np.array([1, 16, -90], dtype=np.float32)
        
        self._randomize_env_level = 7
        self._normalize_obs = True
        self._normalize_act = True
        self._visualize_target = True
        self._reward_method = 1
        self._max_linear_velocity = 1
        self._min_linear_velocity = 0
        self._angular_velocity = 1
        self._minimum_dist_from_target = 0.42
        self._minimum_dist_from_obstacles = 0.26
        
        self._attraction_threshold = 3
        self._attraction_factor = 1
        self._repulsion_threshold = 1
        self._repulsion_factor = 0.1
        self._distance_penalty_factor = 1

        self._num_steps = 0
        self._num_episodes = 0
        


        # TODO : Establish the waypoints in the drone path
        