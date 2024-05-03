from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, Pose
from sensor_msgs.msg import LaserScan, Imu, Range, NavSatFix
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, SetModelState, SetEntityState
from ament_index_python.packages import get_package_share_directory
import math
from functools import partial
import numpy as np
import os

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.get_logger().info("Drone Controller Created")

        # Action Space Publisher:
        #   - Drone Velocity
        self.action_space_vel = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.get_logger().info("Drone Velocity Publisher created!")
        
        # State Space Subscribers:
        #   - IMU
        #   - GPS Nav
        #   - LiDAR (Laser Scan)
        self.state_space_imu = self.create_subscription(Imu, '/demo/imu/out', self.state_space_imu_callback, 1)
        self.state_space_gpsnav = self.create_subscription(Pose, '/demo/gt_pose', self.state_space_gpsnav_callback, 1)
        self.state_space_lidar = self.create_subscription(LaserScan, '/demo/laser_scanner/out', self.state_space_lidar_callback, 1)
        # self.state_space_img = self.create_subscriber(Image, '/demo/front/camera_raw', self.state_space_img_callback, 1)
        # self.state_space_sonar = self.create_subscriber(Range, '/demo/sonar/out', self.state_space_sonar_callback, 1)
        # self.state_space_gpsvel = self.create_subscriber(TwistStamped, '/demo/gps/vel', self.state_space_gpsvel_callback, 1)
        self.get_logger().info("IMU, LiDAR, & GPS Navigation Subscribers created!")
        
        self.client_state = self.create_client(SetEntityState, "/demo/set_entity_state")

        # Initializing Drone State Space Variables
        self._agent_location = np.array([np.float32(1),np.float32(16)])
        self._laser_reads = np.array([np.float32(10)] * 180)

    # Action Space Command Function : Drone Velocity
    def action_space_vel_command(self, velocity):
        """
        Parameter Info:
        velocity = 
                    [[linear_x, linear_y, linear_z],
                     [angular_x, angular_y, angular_z]]

        Message to be sent to the robot is in the geometry_msgs::msg::Twist format, which has both linear and angular components

        NOTE : For now, the drone control parameters are being kept simple: 
                - there will be linear control only in the x and z directions &
                - angular control only about the z axis.
        """
        msg = Twist()
        msg.linear.x = float(velocity[0])
        msg.angular.z = float(velocity[1])
        self.action_space_vel.publish(msg)

        self.get_logger().info("Sent Velocity (Linear X) : " + str(velocity[0]))
        # self.get_logger().info("Sent Velocity (Linear Z) : " + str(velocity[0][3]))
        self.get_logger().info("Sent Velocity (Angular Z) : " + str(velocity[1]))

    # State Space Callback Functions: 
    #   - IMU
    #   - Lidar
    #   - GPS Navigation
    # NOTE : To keep the variables simple, ignoring the GPS Vel & Img sensors
    def state_space_imu_callback(self, msg:Imu):
        self._agent_orientation = 2* math.atan2(msg.orientation.z, msg.orientation.w)
        self._done_orientation = True

    def state_space_lidar_callback(self, msg:LaserScan):
        self._laser_reads = np.array(msg.ranges)
        # Converting All 'inf' measurements to a standardized value of 10
        self._laser_reads[self._laser_reads == np.inf] = np.float32(10)
        self._done_laser = True

    def state_space_gpsnav_callback(self, msg:Pose):
        self._agent_location = np.array([np.float32(np.clip(msg.position.x,-12,12)), np.float32(np.clip(msg.position.y,-35,21))])
        self._done_location = True

# Method to set the state of the robot when an episode ends - /demo/set_entity_state service
    def call_set_robot_state_service(self, robot_pose=[1, 16, -0.707, 0.707]):
        while not self.client_state.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = SetEntityState.Request()
        request.state.name = self.robot_name
        # Pose (position and orientation)
        request.state.pose.position.x = float(robot_pose[0])
        request.state.pose.position.y = float(robot_pose[1])
        request.state.pose.orientation.z = float(robot_pose[2])
        request.state.pose.orientation.w = float(robot_pose[3])
        # Velocity
        request.state.twist.linear.x = float(0)
        request.state.twist.linear.y = float(0)
        request.state.twist.linear.z = float(0)
        request.state.twist.angular.x = float(0)
        request.state.twist.angular.y = float(0)
        request.state.twist.angular.z = float(0)

        future = self.client_state.call_async(request)
        future.add_done_callback(partial(self.callback_set_robot_state))

    # Method that elaborates the future obtained by callig the call_set_robot_state_service method
    def callback_set_robot_state(self, future):
        try:
            response= future.result()
            #self.get_logger().info("The Environment has been successfully reset")
            self._done_set_rob_state = True
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

    # Method to set the state of the target when an episode ends - /demo/set_entity_state service
    def call_set_target_state_service(self, position=[1, 10]):
        while not self.client_state.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = SetEntityState.Request()
        request.state.name = "Target"
        # Pose (position and orientation)
        request.state.pose.position.x = float(position[0])
        request.state.pose.position.y = float(position[1])

        future = self.client_state.call_async(request)
        future.add_done_callback(partial(self.callback_set_target_state))

    # Method that elaborates the future obtained by callig the call_set_target_state_service method
    def callback_set_target_state(self, future):
        try:
            response= future.result()
            #self.get_logger().info("The Environment has been successfully reset")
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))