from rclpy.Node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.get_logger().info("Drone Controller Created")

        # Action Space Publisher : Drone Velocity
        self.action_space_vel = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.get_logger().info("Drone Velocity Publisher created!")
        
        # State Space Subscribers: Images, IMU, Sonar, GPS Navigation, GPS Velocity
        # self.state_space_img = self.create_subscriber(Image, '/simple_drone/front/camera_raw', self.state_space_img_callback, 1)
        
        self.state_space_imu = self.create_subscriber(Imu, '/simple_drone/imu/out', self.state_space_imu_callback, 1)
        self.state_space_gpsnav = self.create_subscriber(NavSatFix, '/simple_drone/gps/nav', self.state_space_gpsnav_callback, 1)
        # self.state_space_sonar = self.create_subscriber(Range, '/simple_drone/sonar/out', self.state_space_sonar_callback, 1)
        # self.state_space_sonar = self.create_subscriber(Range, '/simple_drone/sonar/out', self.state_space_sonar_callback, 1)
        # self.state_space_gpsvel = self.create_subscriber(TwistStamped, '/simple_drone/gps/vel', self.state_space_gpsvel_callback, 1)
        self.get_logger().info("Images, IMU, Sonar, GPS Navigation, GPS Velocity Subscribers created!")

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

        msg.linear.x = float(velocity[0][1])
        # msg.linear.y = float(velocity[0][2])
        msg.linear.z = float(velocity[0][3])

        # msg.angular.x = float(velocity[1][1])
        # msg.angular.y = float(velocity[1][2])
        msg.angular.z = float(velocity[1][3])

        self.action_space_vel.publish(msg)

        self.get_logger().info("Sent Velocity (Linear X) : " + str(velocity[0][1]))
        self.get_logger().info("Sent Velocity (Linear Y) : " + str(velocity[0][3]))
        self.get_logger().info("Sent Velocity (Angular Z) : " + str(velocity[1][3]))

    # State Space Callback Functions: Images, IMU, Sonar, GPS Navigation, GPS Velocity
    # Ignoring the GPS Vel & Img state space for now too keep the variables simple
    #
    def state_space_imu_callback(self, msg:Imu):
        # TODO : Add orientation info of the drone with the IMU msg orientation param
        self._agent_orientation = None

    # def state_space_sonar_callback(self, msg:Range):
    #     # TODO : Add
    #     self._sonar_data = None

    def state_space_gpsnav_callback(self, msg:NavSatFix):
        # TODO : Add postion of the drone with the NavSatFix lat, lon, and altitude params
        self._agent_location = None

    # def state_space_gpsvel_callback(self, msg:TwistStamped):
    #     pass
    # def state_space_img_callback(self, msg:Image):
    #     pass