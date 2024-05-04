# Drone-RL-Control
The GitHub Repository for the Simulation &amp; Control of a Drone using Reinforcement Learning Techniques in a ROS2 Environment.

## Project Description
This repository helps train a quadcopter to find the best way around a path. The robot is equipped with a LiDAR, IMU &amp; GPS sensor.

**Sensor Specifications:** The LiDAR is a 180° laser for obstacle detection with a resolution of 1. The distance measurements from the LiDAR ray have been normalized to a scale of 0 to 10 meters. The IMU is used to check the orientation of the quadcopter. Finally, the GPS sensor is used for finding the position of the robot in the world.

This repository includes the following elements:
- 3D simulation environment of the hospital with the drone.
- A functional OpenAI gymnasium environment designed for training RL agents for motion planning. 
  
## Project Structure: 
The flowchart for the project is as follows:

![Project Flowchart](images/project_flowchart.jpeg)

The directory structure meanwhile is:

```
Drone-RL-Control
|---- sjtu_description
|---- sjtu_drone_control
|---- sjtu_drone_bringup
|---- drone_control
|---- drone_rl
```
Where, the SJTU repository has been used for the drone model & controller.
**NOTE:** The drone model has been modified from its original version to have a LiDAR laser scanner for collecting distance data to the obstacles. This laser scan is being published on the `/<namespace>/laser_scanner/out` topic.

The `drone_control` and `drone_rl` packages are used to interact with the drone in the world and train the model respectively. The generated model is stored in the `drone_rl/models_rl` folder.

## Training the Model
The reinforcement learning model was trained on the [drone_test](/drone_rl/worlds/drone_test.world) gazebo world. The image of this world is:

![Drone Test World](images/drone_test_world.png)

## Dependencies
- ROS2 Humble with Ubuntu 22.04 LTS
- Gazebo integration for ROS2
- OpenAI Gymnasium
- Stable Baselines3
- aws-robomaker-hospital-world (https://github.com/aws-robotics/aws-robomaker-hospital-world/tree/39969a9d250135835cf13b480865647d1a87b6f4/models)
- Tensorboard
- Optuna (for hyperparameters tuning)

## Build Instructions
To train an RL model on the target world with this repository, the following instructions can be followed:

1. Clone this repository inside the src folder of your ROS2 workspace (replace `ros2_ws` with the name of your ROS2 workspace):
```bash
cd ~/ros2_ws/src
git clone https://github.com/melaniayoo/Drone-RL-Control.git
```
2. Build the project and source the installed files
```bash
cd ~/ros2_ws/
colcon build
source ./install/setup.bash
```
3. Launch the world file to load the model into the world:
```bash
ros2 launch drone_rl drone_rl_start.launch.py
```
- And in a separate terminal, start the RL process:
```bash
ros2 launch drone_rl start_training.launch.py
```
This will start moving the drone around the world and generating an RL model.

![Drone Test World](images/drone_training.gif)

## Credits
- Shanghai Jiao Tong University Drone (sjtu_drone) repository (link: https://github.com/NovoG93/sjtu_drone). The description & control packages in the repository are used under the GPL-3.0 license.
- [aws-robotics](https://github.com/aws-robotics/aws-robomaker-hospital-world): The drone test world on which the RL model was trained. Used under the  MIT-0 license.

## Project Members
[Melania Yoo](https://github.com/melaniayoo/) &amp; [Karan Kapoor](https://github.com/k-kaps/)
