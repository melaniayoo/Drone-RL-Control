import rclpy
from rclpy.node import Node
import gymnasium as gym
from gymnasium.envs.registration import register

class DroneTraining(Node):
    def __init__(self):
        super().__init__("drone training", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    
def main():
    rclpy.init()
    node = DroneTraining()
    node.get_logger().info("Drone Training Node Created!")

    model_save_dir = ""

    register(
        id = "SJTU-DroneEnv",
        entry_point = "drone_rl.drone_env:DroneEnv",
        max_episode_steps = 300,
    )

    node.get_logger().info("The drone environment has been registered!")

    