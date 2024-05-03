# Drone-RL-Control
The GitHub Repository for the Simulation &amp; Control of a Drone using Reinforcement Learning Techniques in a ROS2 Environment.

## Project Description
This repository uses different tools like ROS2 Humble, Gazebo, OpenAI Gym, and Stable Baselines3. It helps train agents to find the best way for a robot with a LIDAR sensor to move around and solve path problems. 
The robot implemented is a quadcopter drone with a 180° resolution laser for obstacle detection, an IMU, and a GPS sensor. The LIDAR collects 180 distance measurements that can range from 0 to 10 meters. 

This repository includes the following elements:
- 3D simulation environment of the hospital with the drone.
- A functional Gym environment designed for training RL agents for motion planning. 
  
## Project Structure: 
The flowchart for the project is as follows:

![Project Flowchart](images/project_flowchart.jpeg)

The directory structure meanwhile is:

```
# [TODO] Add the directory structure here
Drone-RL-Control
|---- sjtu_description
|---- sjtu_drone_control
|---- drone_control
|---- drone_rl
...
```

## Training the Model
The reinforcement learning model was trained on the [drone_test](/drone_rl/worlds/drone_test.world) gazebo world. The image of this world is:

![Drone Test World](images/drone_test_world.png)


## Credits
- Shanghai Jiao Tong University Drone (sjtu_drone) repository (link: https://github.com/NovoG93/sjtu_drone). The description & control packages in the repository are used under the GPL-3.0 license.
- [aws-robotics](https://github.com/aws-robotics/aws-robomaker-hospital-world): The drone test world on which the RL model was trained. Used under the  MIT-0 license.

## Project Members
[Melania Yoo](https://github.com/melaniayoo/), Ahnaful Hoque, &amp; [Karan Kapoor](https://github.com/k-kaps/)
