import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen
from std_msgs.msg import Int32

"""
@Author: Elvi Mihai Sabau Sabau
"""
class TurtleSpawnerNode(Node):
    def __init__(self, concurrent_turtles):
        super().__init__('offender_usi_turtle')

        # Create killed array to store the killed offenders
        self.killed = []

        # Create a flag to make spawning an atomic operation and avoid race conditions
        self.spawning = False

        # Initialize the number of turtles and the number of turtles spawned
        self.num_turtles = 0

        # Initialize the number of concurrent turtles
        self.concurrent_turtles = concurrent_turtles

        # Create a service client to spawn turtles
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # Create a subscriber to listen to the killed offender
        self.create_subscription(Int32, '/killedoffender', self.kill_callback, 10)

        # Create a dictionary to store the subscriptions of the spawned turtles and manage their directions
        self.offender_controllers = {}

        # Call the spawn service to spawn initial turtles
        self.spawn_initial_turtles()

        # Create a timer to change the direction of the turtles every 3 seconds
        self.create_timer(1, self.change_direction)

    
    def change_direction(self):
        """ Change the direction of the spawned turtles randomly every second """
        for name, publisher in self.offender_controllers.items():

            # Create a service client to set the pen of the spawned turtle
            set_pen_client = self.create_client(SetPen, f'/{name}/set_pen')
            set_pen_request = SetPen.Request()
            set_pen_request.off = True
            set_pen_client.call_async(set_pen_request)

            # Create a random Twist message to change the direction of the turtle
            cmd_vel = Twist()
            cmd_vel.linear.x = random.uniform(0, 2)
            cmd_vel.angular.z = random.uniform(-1, 1)
            publisher.publish(cmd_vel)


    def kill_callback(self, msg):
        """ Callback function to spawn new turtles when a turtle is killed """

        # Check if the offender is already killed to avoid multiple kills of the same offender ( God forsaken race conditions )
        if msg.data in self.killed:
            return
        
        self.get_logger().info(f'Turtle: offender{msg.data} killed!')
        
        # Add the killed offender to the killed array
        self.killed.append(msg.data)

        # Spawn a new turtle
        self.spawn_turtle()
        


    def spawn_initial_turtles(self):
        """ Spawn initial turtles """
        for _ in range(self.concurrent_turtles):
            future = self.spawn_turtle()
            rclpy.spin_until_future_complete(self, future)


    def spawn_turtle(self):
        """ Spawn offender with random parameters """

        self.get_logger().info(f'Spawning offender{self.num_turtles}')

        # Increment turtle count
        self.num_turtles += 1

        # Create a request to spawn a turtle
        request = Spawn.Request()
        request.x = random.uniform(1, 10)
        request.y = random.uniform(1, 10)
        request.theta = random.uniform(0, 2 * 3.14159)
        request.name = f'offender{self.num_turtles}'

        # Create a publisher to control the spawned turtle
        self.offender_controllers[request.name] = self.create_publisher(Twist, f'/{request.name}/cmd_vel', 10)

        # Call the spawn service to spawn the turtle
        return self.spawn_client.call_async(request)

 
def main():
    rclpy.init()

    # -----------------------------------------------------------------------------------------
    # Spawn 4 turtles alive concurrently
    turtle_spawner_node = TurtleSpawnerNode( concurrent_turtles=4 )
    rclpy.spin(turtle_spawner_node)

if __name__ == '__main__':
    main()