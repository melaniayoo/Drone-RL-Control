from rclpy.Node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TwistStamped

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.get_logger().info("Drone Controller Created")

        # Action Space Publisher : Drone Velocity
        self.action_space_vel = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        
        # State Space Subscribers: Images, IMU, Sonar, GPS Navigation, GPS Velocity
        self.state_space_img = self.create_subscriber(Image, '/simple_drone/front/camera_raw', self.state_space_img_callback, 1)
        self.state_space_imu = self.create_subscriber(Imu, '/simple_drone/imu/out', self.state_space_imu_callback, 1)
        self.state_space_sonar = self.create_subscriber(Range, '/simple_drone/imu/out', self.state_space_sonar_callback, 1)
        self.state_space_gpsnav = self.create_subscriber(NavSatFix, '/simple_drone/gps/nav', self.state_space_gpsnav_callback, 1)
        self.state_space_gpsvel = self.create_subscriber(TwistStamped, '/simple_drone/gps/vel', self.state_space_gpsvel_callback, 1)

    # Action Space Command Function : Drone Velocity
    def action_space_vel_command(self, velocity):
        pass

    # State Space Callback Functions: Images, IMU, Sonar, GPS Navigation, GPS Velocity
    def state_space_img_callback(self, msg:Image):
        pass

    def state_space_imu_callback(self, msg:Imu):
        pass

    def state_space_sonar_callback(self, msg:Range):
        pass

    def state_space_gpsnav_callback(self, msg:NavSatFix):
        pass

    def state_space_gpsvel_callback(self, msg:TwistStamped):
        pass