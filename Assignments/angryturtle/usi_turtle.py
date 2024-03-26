import rclpy
from rclpy.node import Node
from rclpy.task import Future
import sys
from enum import Enum
from math import pow, atan2, sqrt, sin, cos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, SetPen

class Move2GoalNode(Node):
    def __init__(self, usicoords, tolerance, trigger_distance=1.0, kill_distance=0.2):
        # Creates a node with name 'move2goal'
        super().__init__('move2goal')

        # Store the coordinates of the letters and the current letter and coordinate per letter
        self.usicoords = usicoords
        self.usiletter = 0
        self.usilettercoords = 0

        # Tolerance to consider the goal as reached
        self.tolerance = tolerance

        # Distance to trigger the angry state
        self.trigger_distance = trigger_distance

        # Distance to kill the enemy turtle
        self.kill_distance = kill_distance

        # Initialize the current pose of the turtle
        self.current_pose = None

        # Create service clients
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.kill_client = self.create_client(Kill, '/kill')

        # Create a publisher and subscriber
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Get the number of enemy turtles
        self.enemy_turtles = self.get_enemy_turtles()

        # For each offender turtle, create a subscriber of its pose
        for i in self.enemy_turtles:
            self.get_logger().info(f"Subscribing to /offender{i}/pose")
            self.create_subscription(Pose, f'/offender{i}/pose', lambda msg, i=i: self.enemy_pose_callback(i, msg), 10)

        # Store the current enemy turtle ( pos and number of the turtle )
        self.current_enemy_pos = None
        self.current_enemy_id = None

        # Disable writing at the beginning
        pen_req = SetPen.Request()
        pen_req.off = True
        self.pen_client.call_async(pen_req)

        # Current state of the turtle ( WRITING, ANGRY, RETURNING )
        self.state = "WRITING"

    def start_moving(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(0.1, self.move_callback)
        
        # Create a Future object which will be marked as "completed" once the turtle reaches the goal
        self.done_future = Future()
        
        return self.done_future
        
    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)

    def enemy_pose_callback(self, offender_id, msg):
        """Check if the turtle is close to a enemy turtle"""
        print(f"Enemy turtle {offender_id} is in pos ({msg.x}, {msg.y})")

        if self.state == "WRITING" or self.state == "ANGRY" and self.euclidean_distance(self.current_pose, msg) <= self.trigger_distance:
            self.get_logger().info("Enemy turtle detected, changing to ANGRY state.")
            self.state = "ANGRY"
            self.current_enemy_pos = msg
            self.current_enemy_id = offender_id

            # Stop writing
            pen_req = SetPen.Request()
            pen_req.off = True
            self.pen_client.call_async(pen_req)

        
    def move_callback(self):
        """Callback called periodically by the timer to publish a new command."""

        if self.state == "WRITING":
            self.normal_behaviour()
        elif self.state == "ANGRY":
            self.angry_behaviour()
        elif self.state == "RETURNING":
            self.return_behaviour()


    def normal_behaviour(self):
        """Move to the next coordinate of the letter, and write it."""

        # Pose of the next coordinate
        nextcoords = Pose()
        nextcoords.x = float(self.usicoords[self.usiletter][self.usilettercoords][0])
        nextcoords.y = float(self.usicoords[self.usiletter][self.usilettercoords][1])

        # Writing handler
        pen_req = SetPen.Request()

        # Create a Twist message to send commands to the turtle
        cmd_vel = Twist()

        # Wait until we receive the current pose of the turtle for the first time
        if self.current_pose is None:
            return
        
        # For each offender turtle
        if self.euclidean_distance(nextcoords, self.current_pose) >= self.tolerance:

            # Compute the velocities to send to the turtle
            cmd_vel.linear.x = self.linear_vel(nextcoords, self.current_pose, 2)
            cmd_vel.angular.z = self.angular_vel(nextcoords, self.current_pose, 10)
            
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        else:
            self.get_logger().info("Coordinate reached, moving to next coordinate")
            
            # Stop the turtle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.vel_publisher.publish(cmd_vel)

            # Move to the next coordinate
            if self.usilettercoords < len(self.usicoords[self.usiletter])-1:
                self.usilettercoords += 1

                # Enable writing by setting the pen color to white and its width
                pen_req.off = False
                pen_req.width = 5
                pen_req.r = 255
                pen_req.g = 255
                pen_req.b = 255
                self.pen_client.call_async(pen_req)
            else:
                self.get_logger().info("Letter finished, moving to next letter")
                
                # Disable writing
                pen_req.off = True
                self.pen_client.call_async(pen_req)

                # Reset letter coordinates
                self.usilettercoords = 0
            
                # Move to the next letter
                if self.usiletter < len(self.usicoords)-1:
                    self.usiletter += 1
                # If all letters are finished, mark the future as completed
                else:
                    self.done_future.set_result(True)


    def angry_behaviour(self):
        """Move to the enemy turtle and kill it"""
        # Create a Twist message to send commands to the turtle
        cmd_vel = Twist()

        # Check if the enemy turtle is close enough to kill it
        if self.euclidean_distance(self.current_enemy_pos, self.current_pose) >= self.kill_distance:
            self.get_logger().info(f"Moving to enemy turtle {self.current_enemy_id} >:)")


            # Compute the velocities to send to the turtle
            cmd_vel.linear.x = self.linear_vel(self.current_enemy_pos, self.current_pose, 2)
            cmd_vel.angular.z = self.angular_vel(self.current_enemy_pos, self.current_pose, 6)
            
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        else:
            self.get_logger().info("Enemy turtle reached, killing it")
            
            # Stop the turtle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.vel_publisher.publish(cmd_vel)

            # Kill the enemy turtle
            kill_req = Kill.Request()
            kill_req.name = f'offender{self.current_enemy_id}'
            self.kill_client.call_async(kill_req)

            # Return to the previous state
            self.state = "WRITING"

    def return_behaviour(self):
        """Return to the original letter coordinate, and continue writing"""

        

    def get_enemy_turtles(self):
        """Return an array with the number of all enemy turtles"""

        # Get the list of topics
        topics = self.get_topic_names_and_types()

        # Filter the topics that start with '/offender'
        offender_topics = [topic for topic, _ in topics if topic.startswith('/offender')]
        
        # Get the number of enemy turtles
        enemy_turtles = []
        for topic in offender_topics:
            if topic.startswith('/offender'):
                turtle_number = int(topic.split('offender')[1].split('/')[0])
                if turtle_number not in enemy_turtles:
                    enemy_turtles.append(turtle_number)
        
        self.get_logger().info(f"Enemy turtles: {enemy_turtles}")
        
        return enemy_turtles



    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - current_pose.x), 2) +
                    pow((goal_pose.y - current_pose.y), 2))

    def angular_difference(self, goal_theta, current_theta):
        """Compute shortest rotation from orientation current_theta to orientation goal_theta"""
        return atan2(sin(goal_theta - current_theta), cos(goal_theta - current_theta))

    def linear_vel(self, goal_pose, current_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)

    def angular_vel(self, goal_pose, current_pose, constant=10):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        goal_theta = self.steering_angle(goal_pose, current_pose)
        return constant * self.angular_difference(goal_theta, current_pose.theta)


def main():
    
    u = [(1, 10), (1.0, 6.0), (1, 2), (2.0, 2.0), (3, 2), (3.0, 6.0), (3, 10)]
    s = [(7, 10), (6.0, 10.0), (5, 10), (5.0, 8.0), (5, 6), (6.0, 6.0), (7, 6), (7.0, 4.0), (7, 2), (6.0, 2.0), (5, 2)]
    i = [(9, 10), (9, 2)]
    usicoords = [u, s, i]
    
    tolerance = 0.1
    trigger_distance = 1.0
    kill_distance = 0.2

    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = Move2GoalNode(usicoords, tolerance, trigger_distance, kill_distance)
    done = node.start_moving()
    
    # Keep processings events until the turtle has reached the goal
    rclpy.spin_until_future_complete(node, done)
    
    # Alternatively, if you don't want to exit unless someone manually shuts down the node
    # rclpy.spin(node)

if __name__ == '__main__':
    main()


