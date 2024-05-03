import rclpy
from rclpy.node import Node
import gymnasium as gym
from gymnasium.envs.registration import register
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.monitor import Monitor

class DroneTraining(Node):
    def __init__(self):
        super().__init__("drone training", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    
def main():
    # Defining some constant variables
    REWARD_THRESHOLD = 9000
    VERBOSE = 1
    EVAL_FREQ = 100000
    EVAL_EPISODES = 40
    N_STEPS = 20480
    GAMMA = 0.9880614935504514
    GAE_LAMBDA = 0.9435887928788405
    ENT_COEF = 0.00009689939917928778
    VF_COEF = 0.6330533453055319
    LEARNING_RATE = 0.00001177011863371444
    CLIP_RANGE = 0.1482

    rclpy.init()
    node = DroneTraining()
    node.get_logger().info("Drone Training Node Created!")

    model_save_dir = ""
    log_save_dir = ""

    register(
        id = "SJTU-DroneEnv",
        entry_point = "drone_rl.drone_env:DroneEnv",
        max_episode_steps = 300,
    )

    node.get_logger().info("The drone environment has been registered!")

    drone_env = gym.make("SJTU-DroneEnv")
    drone_env = Monitor(drone_env)

    stop_callback = StopTrainingOnRewardThreshold(REWARD_THRESHOLD, VERBOSE)
    eval_callback = EvalCallback(drone_env, callback_on_new_best=stop_callback,
                                 eval_freq=EVAL_FREQ, best_model_save_path=model_save_dir, n_eval_episodes=EVAL_EPISODES)
    
    model = PPO("MultiInputPolicy", drone_env, verbose=VERBOSE, tensorboard_log=log_save_dir, n_steps=N_STEPS, 
                gamma=GAMMA, gae_lambda=GAE_LAMBDA, ent_coef=ENT_COEF, vf_coef=VF_COEF, learning_rate=LEARNING_RATE, clip_range=CLIP_RANGE)
    
    try:
        model.learn(total_timesteps=int(40000000), reset_num_timesteps=False, callback=eval_callback, tb_log_name="PPO_test")
    except KeyboardInterrupt:
        model.save(f"{model_save_dir}/PPO_test")
    
    model.save(f"{model_save_dir}/PPO_test")