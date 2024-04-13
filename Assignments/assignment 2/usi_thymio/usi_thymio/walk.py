import rclpy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.node import Node

"""
Author: Elvi Mihai Sabau Sabau
"""

class WalkController(Node):
    """ Walk until an obstacle is detected, then turn in the direction with no obstacle"""

    def __init__(self):
        super().__init__('walk_controller')

        #  --- Varialbes ---
        # Initialize proximity sensors
        self.prox_sensors = {'prox1': -1.0, 'prox2': -1.0, 'prox3': -1.0, 'prox4': -1.0}
        
        # Speed vel
        self.speed = 0.5

        # Turn vel
        self.turn = 1.0

        # Val used to generate a random direction ( used for uncertain conditions and to avoid loops )
        self.uncertain = False

        # Random direction
        self.random_direction = 1

        # Threshold of how close should the robot be to an obstacle to consider it detected
        self.threshold = 0.03


        # --- Publishers and Subscribers ---
        # Create a publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to get proximity sensor readings
        self.prox1_subscriber = self.create_subscription(Range, 'proximity/left', lambda msg: self.prox_callback(msg, 'prox1'), 10)
        self.prox2_subscriber = self.create_subscription(Range, 'proximity/center_left', lambda msg: self.prox_callback(msg, 'prox2'), 10)
        self.prox3_subscriber = self.create_subscription(Range, 'proximity/center_right', lambda msg: self.prox_callback(msg, 'prox3'), 10)
        self.prox4_subscriber = self.create_subscription(Range, 'proximity/right', lambda msg: self.prox_callback(msg, 'prox4'), 10)


        # --- Timers ---
        # Control loop timer
        self.create_timer(0.1, self.control_loop)


    # --- Control Loop ---
    def control_loop(self):
        """Control loop to move the robot based on proximity sensor readings"""

        # Get sensor readings
        prox1, prox2, prox3, prox4 = self.prox_sensors['prox1'], self.prox_sensors['prox2'], self.prox_sensors['prox3'], self.prox_sensors['prox4']

        # Get the maximum proximity sensors from each side
        prox_left = max(prox1, prox2)
        prox_right = max(prox3, prox4)

        # Move forward if no obstacles detected
        if all(sensor == -1.0 for sensor in [prox_left, prox_right]):
            self.uncertain = False
            self.get_logger().info("No obstacles detected. Moving forward.")
            self.move(self.speed, 0.0)
            return

        # Obstacle on the left or center-left
        if prox_right == -1.0 and prox_left < self.threshold:
            self.uncertain = False
            self.get_logger().info("Obstacle detected on the left. Turning left.")
            self.move(0, -self.turn)  # Turn right
            return

        # Obstacle on the right or center-right
        if prox_left == -1.0 and prox_right < self.threshold:
            self.uncertain = False
            self.get_logger().info("Obstacle detected on the right. Turning right.")
            self.move(0, self.turn)  # Turn left
            return
        
        # Try to squeeze through the middle
        if prox2 == -1.0 and prox3 == -1.0 and prox1 < self.threshold and prox4 < self.threshold:
            self.uncertain = False
            self.get_logger().info("Squeezing through the middle. Moving forward.")
            self.move(self.speed, 0.0)
            return
        
        # If the robot got here, and uncertain conditions are not set, set them and generate a random direction ( 1 , -1)
        if not self.uncertain:
            self.uncertain = True
            self.random_direction = random.choice([-1, 1])
            self.get_logger().info("Uncertain conditions O.o. Generating random direction.")
            return              
        
        # Default behavior: turn right if surrounded or uncertain conditions
        self.get_logger().info("Uncertain conditions O.o. Turning in a random direction.")
        self.move(0.0, -self.turn * self.random_direction)  


    # --- Callbacks ---
    # Proximity sensor callback
    def prox_callback(self, msg, sensor_id):
        """Callback function for proximity sensors"""
        self.prox_sensors[sensor_id] = round(msg.range, 4)
        self.get_logger().info(f"Proximity sensors: {self.prox_sensors}")


    # --- Movements Functions ---
    def move(self, speed, turn):
        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = float(turn)
        self.velocity_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WalkController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
