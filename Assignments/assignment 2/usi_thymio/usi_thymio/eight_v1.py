import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class EightController(Node):
    def __init__(self):
        super().__init__('eight_controller')

        # Create a publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Start State: Forward 1, then Half Circle Right, Forward 2, Half Circle Left and back to Forward 1
        self.state = 'forward_1'

        # Create a timer to control the robot
        self.create_timer(5, self.control_loop)

    def control_loop(self):
        """ Control the robot based on the state """
        self.stop()

        if self.state == 'forward_1':
            self.get_logger().info('Forward 1')
            self.move(speed=0.1)
            self.state = 'half_circle_right'
        elif self.state == 'half_circle_right':
            self.get_logger().info('Half Circle Right')
            self.move(speed=0.1, rads=-1)
            self.state = 'forward_2'
        elif self.state == 'forward_2':
            self.get_logger().info('Forward 2')
            self.move(speed=0.1)
            self.state = 'half_circle_left'
        elif self.state == 'half_circle_left':
            self.get_logger().info('Half Circle Left')
            self.move(speed=0.1, rads=1)
            self.state = 'forward_1'
        

    def move(self, speed=0.0, rads=0.0):
        """ Move the robot with a given speed and turn """
        velocity = Twist()
        
        # Forward Speed
        velocity.linear.x = float(speed)

        # Rotate speed
        velocity.angular.z = float(rads)

        self.velocity_publisher.publish(velocity)

    def stop(self):
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
