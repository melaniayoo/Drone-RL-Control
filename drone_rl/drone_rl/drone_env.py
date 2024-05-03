import rclpy
from gymnasium import Env
import numpy as np
from gymnasium.spaces import Dict, Box
import math
from drone_control.drone_control import DroneController

class DroneEnv(DroneController, Env):
    """
    Defines the:
        - Action Space
        - Observation Space
        - Target Location
    """
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

        self.get_logger().info("Initialized SJTU Drone Environment With:")
        self.get_logger().info("Initial Target Location: " + str(self._target_location))
        self.get_logger().info("Initial Agent Location: " + str(self._initial_agent_location))
        self.get_logger().info("Maximum Linear Velocity: " +  str(self._max_linear_velocity))

        # TODO : Define action space and observation space Gym Structures

        self.action_space = None
        self.observation_space = None

        # TODO : Define the Waypoints in the Map for the Drone
        self.robot_location = None
        self.waypoints_location = None
        self.eval_locations = None

        self._which_waypoint = 0
        self._successes = 0
        self._failures = 0
        self._completed_paths = 0

    def step(self, action):
        self.step += 1

        if self._normalize_act == True:
            action = self.denormalize_action(action)

        self.action_space_vel_command(action)

        self.spin()

        # TODO : Compute Statistics for the RL Model      

    def render(self):
        pass

    def reset(self, seed = None, options = None):
        pass

    def _get_obs(self):
        pass

    def compute_rewards(self, info):

        if self._reward_method == 0:
            if (info["distance"] < self._minimum_dist_from_target):
                reward = 1
                self.get_logger().info("TARGET REACHED")
            elif (any(info["laser"] < self._minimum_dist_from_obstacles)):
                reward = -1
                self.get_logger().info("HIT AN OBSTACLE")        
            else:
                reward = 0
        elif self._reward_method == 1:
            if (info["distance"] < self._minimum_dist_from_target):
                reward = 1
                self.get_logger().info("TARGET REACHED")
            elif (any(info["laser"] < self._minimum_dist_from_obstacles)):
                reward = -0.1
                self.get_logger().info("HIT AN OBSTACLE")
            else:
                reward = 0

        return reward    