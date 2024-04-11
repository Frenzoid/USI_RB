import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

"""
Author: Elvi Mihai Sabau Sabau
"""

class EightController(Node):
    def __init__(self):
        super().__init__('eight_controller')

        # Create a publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Start State: circle_right then circle_left
        self.state = 'circle_right'

        # Setup speed and rads ( max speed: 14cm/s (values from: 0 - 1), max rads: 3 rad/s (values from: 0 - 3) )
        self.speed = 0.2
        self.rads = 2.0

        # Setup time necessary to complete a circle
        self.circle_time = 7

        # Start the robot
        self.control_loop()

        # Create a timer to change the state every circle_time
        self.create_timer(self.circle_time, self.control_loop)

    def control_loop(self):
        """ Control the robot based on the state """

        if self.state == 'circle_right':
            self.get_logger().info('Circle Right')
            self.move(self.speed, -self.rads)
            self.state = 'circle_left'
        elif self.state == 'circle_left':
            self.get_logger().info('Circle Left')
            self.move(self.speed, self.rads)
            self.state = 'circle_right'
        

    # --- Movements Functions ---
    def move(self, speed=0.0, rads=0.0):
        """ Move the robot with a given speed and turn """
        velocity = Twist()
        
        # Forward Speed
        velocity.linear.x = float(speed)

        # Rotate speed
        velocity.angular.z = float(rads)

        self.velocity_publisher.publish(velocity)

    def stop(self):
        """ Stop the robot """
        self.velocity_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    
    # movemments based on states
    node = EightController()

    rclpy.spin(node)

    # Detect ctrl+c and stop the robot
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
