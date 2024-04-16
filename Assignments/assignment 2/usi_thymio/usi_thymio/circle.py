import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations

"""
Author: Elvi Mihai Sabau Sabau
"""

class CircleController(Node):
    def __init__(self):
        super().__init__('circle_controller')

        # Create a publisher to send velocity commands
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.move(1, 1)
        

    # --- Movements Functions ---
    def move(self, speed=0.0, rads=0.0):
        """ Move the robot with a given speed and turn """
        self.get_logger().info("Moving the robot with speed: {:.2f} and rads: {:.2f}".format(speed, rads), throttle_duration_sec=0.4)
        velocity = Twist()
        
        # Forward Speed
        velocity.linear.x = float(speed)

        # Rotate speed
        velocity.angular.z = float(rads)

        self.velocity_publisher.publish(velocity)


    # --- Callbacks ---
    def odom_callback(self, msg):
        """ Callback for the odometry topic, here we get a 3d pose, and we convert it to a 2d pose (x, y, theta)"""
        pose2d = self.pose3d_to_2d(msg.pose.pose)
        self.pose = pose2d
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.4
        )

    # --- Helper Functions ---
    def pose3d_to_2d(self, pose3d):
        """ Convert a 3D pose to a 2D pose """

        quaternion = (
            pose3d.orientation.x,
            pose3d.orientation.y,
            pose3d.orientation.z,
            pose3d.orientation.w
        )
        
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        return (
            pose3d.position.x,  # x position
            pose3d.position.y,  # y position
            yaw                 # theta orientation
        )

def main(args=None):
    rclpy.init(args=args)
    
    # movemments based on states
    node = CircleController()

    rclpy.spin(node)

    # Detect ctrl+c and stop the robot
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
