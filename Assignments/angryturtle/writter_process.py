import rclpy
import sys
import math
import numpy as np
from rclpy.node import Node
from rclpy.task import Future
from math import pow, atan2, sqrt, sin, cos
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int32
from turtlesim.srv import Kill, SetPen

class TurtleWriterNode(Node):
    def __init__(self, usicoords, tolerance, trigger_distance=1.0, kill_distance=0.2, lookahead=0.0):
        # Creates a node with name 'move2goal'
        super().__init__('writer_usi_turtle')

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

        # Lookahead distance
        self.lookahead = lookahead

        # Initialize the current pose of the turtle
        self.current_pose = None

        # Store the previous pose of the turtle before the angry state
        self.previous_pose = None

        # Create service clients
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.kill_client = self.create_client(Kill, '/kill')

        # List of current enemy turtles
        self.enemy_turtles = []

        # Create subscriber array for enemy turtles
        self.enemy_turtles_subscriptions = {}

        # Create a publisher and subscriber
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.offenderkilled_publisher = self.create_publisher(Int32, '/killedoffender', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Store the current enemy turtle ( pos and number of the turtle )
        self.current_enemy_pos = None
        self.current_enemy_id = None

        # Disable writing at the beginning
        pen_req = SetPen.Request()
        pen_req.off = True
        self.pen_client.call_async(pen_req)
        
        # Current state of the turtle ( WRITING, ANGRY, RETURNING )
        self.state = "WRITING"


    def check_for_enemies(self):
        """Callback functions which checks if theres new enemy turtles and creates a subscriber for them"""
        
        # Get the number of enemy turtles
        new_enemy_turtles = self.get_enemy_turtles()

        # For each new enemy turtle, create a subscriber of its pose
        for i in new_enemy_turtles:
            if i not in self.enemy_turtles:
                self.get_logger().info(f"Subscribing to /offender{i}/pose")
                self.enemy_turtles_subscriptions[i] = self.create_subscription(Pose, f'/offender{i}/pose', lambda msg, i=i: self.enemy_pose_callback(i, msg), 10)
        
        # Update the list of enemy turtles
        self.enemy_turtles = new_enemy_turtles


    def start_moving(self):
        # Create a callback to check for new enemy turtles
        self.create_timer(0.1, self.check_for_enemies)
        
        # Create and immediately start a timer that will regularly publish commands
        self.create_timer(0.1, self.move_callback)
        
        # Create a Future object which will be marked as "completed" once the turtle reaches the goal
        self.done_future = Future()
        
        # Return the Future object to the main function
        return self.done_future
    
        
    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        # Store the current pose of the turtle
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)


    def enemy_pose_callback(self, offender_id, msg):
        """Check if the turtle is close to a enemy turtle"""

        # If the turtle is writing or angry and the enemy turtle is close enough, change to ANGRY state
        if self.state == "WRITING" or self.state == "RETURNING" and self.euclidean_distance(self.current_pose, msg) <= self.trigger_distance:

            self.get_logger().info("Enemy turtle detected, changing to ANGRY state.")
            self.get_logger().info(f"Enemy turtle {offender_id} is in pos ({msg.x}, {msg.y})")

            # Store current pose before changing to ANGRY state if this is the first time ( From WRITING to ANGRY )
            if self.state == "WRITING":
                self.previous_pose = self.current_pose

            # Change to ANGRY state
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
            self.returning_behaviour()


    def normal_behaviour(self):
        """Move to the next coordinate of the letter, and write it."""

        # Pose of the next coordinate
        nextcoords = Pose()
        nextcoords.x = float(self.usicoords[self.usiletter][self.usilettercoords][0])
        nextcoords.y = float(self.usicoords[self.usiletter][self.usilettercoords][1])

        # Writing handler
        pen_req = SetPen.Request()

        # Wait until we receive the current pose of the turtle for the first time
        if self.current_pose is None:
            return
        
        # For each offender turtle
        if self.euclidean_distance(nextcoords, self.current_pose) >= self.tolerance:

            # Compute the velocities to send to the turtle
            cmd_vel = self.get_objective_speeds(nextcoords, self.current_pose, 2, 10, self.lookahead)
            
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        else:
            self.get_logger().info("Coordinate reached, moving to next coordinate")
            
            # Stop the turtle
            cmd_vel = Twist()
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
                # If all letters are finished, reset the letter counter
                else:
                    self.usiletter = 0
                    self.usilettercoords = 0


    def angry_behaviour(self):
        """Move to the enemy turtle and kill it"""

        # Check if the enemy turtle is close enough to kill it
        if self.euclidean_distance(self.current_enemy_pos, self.current_pose) >= self.kill_distance:
            self.get_logger().info(f"Moving to enemy turtle {self.current_enemy_id} >:)")


            # Compute the velocities to send to the turtle
            cmd_vel = self.get_objective_speeds(self.current_enemy_pos, self.current_pose, 2, 10, self.lookahead)
            
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        else:
            self.get_logger().info("Enemy turtle reached, killing it")
            
            # Stop the turtle
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.vel_publisher.publish(cmd_vel)

            # Kill the enemy turtle
            kill_req = Kill.Request()
            kill_req.name = f'offender{self.current_enemy_id}'
            self.kill_client.call_async(kill_req)
            self.offenderkilled_publisher.publish(Int32(data=self.current_enemy_id))
            self.current_enemy_pos = None
            self.current_enemy_id = None

            # Return to the previous state
            self.state = "RETURNING"

    def returning_behaviour(self):
        """Return to the previous state after killing the enemy turtle"""

        # Check if the turtle is close enough to the previous position
        if self.euclidean_distance(self.previous_pose, self.current_pose) >= self.tolerance:
            self.get_logger().info("Returning to previous position O.o")

            # Compute the velocities to send to the turtle
            cmd_vel = self.get_objective_speeds(self.previous_pose, self.current_pose, 2, 10, self.lookahead)
            
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        else:
            self.get_logger().info("Previous position reached, returning to previous state")
            
            # Stop the turtle
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.vel_publisher.publish(cmd_vel)

            # Return to the previous state
            self.state = "WRITING"
        

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
        
        if len(enemy_turtles) > 0:
            self.get_logger().info(f"Enemy turtles: {enemy_turtles}")
        
        return enemy_turtles


    def get_objective_speeds(self, goal_pose, current_pose, linear_speed, angular_speed, lookahead=False):
        """Compute the objective speeds to reach the goal, return Twist."""
        cmd_vel = Twist()

        if lookahead and self.current_enemy_pos is not None:
            meters_ahead = self.current_enemy_pos.linear_velocity / 2 * self.euclidean_distance(self.current_enemy_pos, current_pose)

            # clipping the meters ahead to be between -5 and 5 to avoid going too far ahead or behind
            meters_ahead = np.clip(meters_ahead, -5, 5)

            # computing the versor of the enemy turtle
            versor_x = np.cos(self.current_enemy_pos.theta)
            versor_y = math.sqrt(1 - math.pow(versor_x, 2))

            # computing the pose ahead wrt the origin
            pose_ahead_wrt_origin = meters_ahead * np.array([versor_x, versor_y])

            # computing the goal pose ahead wrt the origin
            goal_pose.x = pose_ahead_wrt_origin[0] + self.current_enemy_pos.x
            goal_pose.y = pose_ahead_wrt_origin[1] + self.current_enemy_pos.y
            goal_pose.theta = self.current_enemy_pos.theta

        cmd_vel.linear.x = self.linear_vel(goal_pose, current_pose, linear_speed)
        cmd_vel.angular.z = self.angular_vel(goal_pose, current_pose, angular_speed)
        
        return cmd_vel


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
    
    # Define the coordinates of the letters
    u = [(1, 10), (1.0, 6.0), (1, 2), (2.0, 2.0), (3, 2), (3.0, 6.0), (3, 10)]
    s = [(7, 10), (6.0, 10.0), (5, 10), (5.0, 8.0), (5, 6), (6.0, 6.0), (7, 6), (7.0, 4.0), (7, 2), (6.0, 2.0), (5, 2)]
    i = [(9, 10), (9, 2)]
    usicoords = [u, s, i]
    
    # Define the tolerance, trigger distance and kill distance
    tolerance = 0.1
    trigger_distance = 3.0 # k2
    kill_distance = 0.5    # k1
    lookahead = False

    # Initialize the ROS client library
    rclpy.init(args=sys.argv)


    # -----------------------------------------------------------------------------------------
    # Create an instance of your node class
    node = TurtleWriterNode(usicoords, tolerance, trigger_distance, kill_distance, lookahead)
    node.start_moving()
        
    # Alternatively, if you don't want to exit unless someone manually shuts down the node
    rclpy.spin(node)

if __name__ == '__main__':
    main()

