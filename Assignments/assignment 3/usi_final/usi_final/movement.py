import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import mediapipe as mp
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

"""
Author: Elvi Mihai Sabau Sabau
"""

class PrimaryController(Node):
    """ Get camera and publish finger number """
    def __init__(self):
        super().__init__('Primary_controllers')

        # --- Configuration Variables ---
        # Interval time used to measure what movement / direction should we do
        self.interval_time = 0.5

        # Speeds at which the robot should move
        self.velocity_speed = 0.3
        self.direction_speed = 0.6

        # --- Global variables ---
        # Array of movement commands received from the past 1s
        self.movement = []

        # Array of direction commands received from the past 1s.
        self.direction = []


        # --- Subscribers and Publishers ---
        # Subscribe to the topic movement (movement, direction)
        self.movement_subscriber = self.create_subscription(String, '/cmd_command', self.cmd_command_callback, 10)

        # Rbomaster velocity publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)


        # --- Timers ---
        self.get_logger().info("Starting the timer")
        # Timer to check the movement and direction, and perform the action
        self.timer = self.create_timer(self.interval_time, self.perform_action)


    # --- Main Functions ---
    def perform_action(self):
        """ Perform the action based on the movement and direction """

        # If there is no movement or direction, do nothing
        if len(self.movement) == 0 or len(self.direction) == 0:
            self.move(speed=0.0, rads=0.0)
            return
        
        # Get the most common movement and direction, only integers
        avg_mvm = np.mean(self.movement)
        avg_dir = np.mean(self.direction)

        # Reset the movement and direction
        self.movement = []
        self.direction = []

        # Perform the action
        self.move(speed = avg_mvm * self.velocity_speed, 
                   rads = avg_dir * self.direction_speed)        


    # --- Callbacks ---    
    def cmd_command_callback(self, msg):
        """ Append the movement to the movement array """

        # Split the message into movement and direction
        direction, movement = msg.data.split(",")

        # Turn movement and direction to float
        movement = float(movement)
        direction = float(direction)

        # Append the movement and direction to the array
        self.movement.append(movement)
        self.direction.append(direction)

        # Forward Speed
        self.get_logger().info("Movement received: {}".format(msg.data))



    # --- Helper Functions ---
    def move(self, speed=0.0, rads=0.0, side=0.0):
        """ Move the robot with a given speed and turn """
        self.get_logger().info("Moving the robot with speed: {:.2f} and rads: {:.2f}".format(speed, rads), throttle_duration_sec=0.4)
        velocity = Twist()
        
        # Forward Speed
        velocity.linear.x = float(speed)

        # Rotate speed
        velocity.angular.z = float(rads)

        # Side displacement speed
        velocity.linear.y = float(side)

        self.velocity_publisher.publish(velocity)
    
# --- Main ---
def main(args=None):
    rclpy.init(args=args)
    node = PrimaryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
