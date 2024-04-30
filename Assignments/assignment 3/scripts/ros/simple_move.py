import rclpy
from rclpy.node import Node
import cv2
from geometry_msgs.msg import Twist
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

"""
Author: Elvi Mihai Sabau Sabau
"""

class SecondaryController(Node):
    """ Get camera and publish finger number """
    def __init__(self):
        super().__init__('Secondary_controllers')

        # Create a publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Move Forward ( kinda redundant since were sending the same message every second, but just for the sake of the timer example )
        self.create_timer(1, lambda: self.move(1, 1))

    
    def move(self, speed=0.0, rads=0.0):
        """ Move the robot with a given speed and turn """
        self.get_logger().info("Moving the robot with speed: {:.2f} and rads: {:.2f}".format(speed, rads), throttle_duration_sec=0.4)
        velocity = Twist()
        
        # Forward Speed
        velocity.linear.x = float(speed)

        # Rotate speed
        velocity.angular.z = float(rads)

        self.velocity_publisher.publish(velocity)
    
# --- Main ---
def main(args=None):
    rclpy.init(args=args)
    node = SecondaryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
