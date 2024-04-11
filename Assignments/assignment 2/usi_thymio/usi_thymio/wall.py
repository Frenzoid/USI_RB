import rclpy
from math import sqrt, pow

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import tf_transformations


"""
Author: Elvi Mihai Sabau Sabau
"""

class WallController(Node):
    def __init__(self):
        super().__init__('wall_controller')

        # Publishers
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.prox1_subscriber = self.create_subscription(Range, 'proximity/left', lambda msg: self.prox_callback(msg, 'prox1'), 10)
        self.prox2_subscriber = self.create_subscription(Range, 'proximity/center_left', lambda msg: self.prox_callback(msg, 'prox2'), 10)
        self.prox3_subscriber = self.create_subscription(Range, 'proximity/center_right', lambda msg: self.prox_callback(msg, 'prox3'), 10)
        self.prox4_subscriber = self.create_subscription(Range, 'proximity/right', lambda msg: self.prox_callback(msg, 'prox4'), 10)

        # Setup speed and rads ( max speed: 14cm/s (values from: 0 - 1), max rads: 3 rad/s (values from: 0 - 3) )
        self.speed = 0
        self.rads = 0

        # Setup the pose of the robot (x, y, theta ( yaw ))
        self.pose = None

        # Pose variable to store the pose when we face the wall and also the pose when we face away from the wall ( used to know how may degrees we have to turn, and also
        #   to know when we have traversed 2 meters )
        self.previous_pose = None

        # Setup loop time
        self.loop_time = 0.1

        # Amount of meters we want to move forward
        self.travel_distance = 2


        # --- Tresholds --- (0.1 = 10cm)
        # Threshold error for when we initially approach the wall ( how close the robot should be to the wall because the sensors are bad >:c (max: 12cm) )
        self.approach_threshold = 0.03

        # Threshold error for the centering of the robot ( diff beteween sensors )
        self.centering_threshold = 0.001

        # Threshold error for the turn around ( how similar the back sensors should be )
        # Threshold error for the turn around action ( in radians )
        self.turn_around_threshold = 0.5


        # Current state of the robot
        self.state = 'FORWARD'

        # Store 4 proximity sensors ( we use 4 separate variables because the odometry data comes asincronously, and we want to avoid overwritting )
        #  we also have a topic for a "center" sensor, but its just the average of the 4 sensors ( i think ), still we need both central sensors
        #  to face forward the MyT. ( -1.0 means nothing detected and anything else means something detected ( min distance: 12cm ))
        self.prox1 = -1.0 # left
        self.prox2 = -1.0 # center_left
        self.prox3 = -1.0 # center_right
        self.prox4 = -1.0 # right

        # Store the back sensors
        self.bprox1 = -1.0 # back_left
        self.bprox2 = -1.0 # back_right

        # Start the robot
        self.control_loop()

        # Create a timer to change the state every circle_time
        self.create_timer(self.loop_time, self.control_loop)

    # --- Main Control Loop ---
    def control_loop(self):
        """ Control the robot based on the state """

        # Wait until we have odometry data
        if self.pose is None:
            self.get_logger().info("Waiting for odometry data...")
            return
        
        self.get_logger().info(f"Status: {self.state}", throttle_duration_sec=0.4)

        # If the state is forward, we move forward and check the proximity sensors
        if self.state == 'FORWARD':
            self.forward_behaviour()
        if self.state == 'FACE_WALL':
            self.face_wall_behaviour()
        if self.state == 'TURN_AROUND':
            self.turn_around_behaviour()
        if self.state == 'FORWARD_2M':
            self.forward_2m_behaviour()
            

    # --- States Behaviours ---
    def forward_behaviour(self):
        """ Behaviour of the robot when it is in the forward state """

        # If all proximity sensors detect nothing, move forward, else stop and change the state to FACE_WALL
        if all(prox == -1.0 for prox in [self.prox1, self.prox2, self.prox3, self.prox4]):
            self.move(speed=1, rads=0.0)
        else:
            self.stop_movement()
            self.state = 'FACE_WALL'
            return

    def face_wall_behaviour(self):
        """ Behaviour of the robot when it is in the face wall state """

        # Check if any proximity sensor is close enough to the wall for the sensor to get a good reading, compare with the closest sensor
        #  if not close enough, move forward slowly 
        prox_values = [prox for prox in [self.prox1, self.prox2, self.prox3, self.prox4] if prox != -1.0]
        if prox_values and min(prox_values) > self.approach_threshold:
            self.get_logger().info("Approaching the wall slooooowly O.o")
            self.move(speed=0.1, rads=0.0)
            return

        # Get the rotation sign ( -1 or 1 ) based on the proximity sensors
        rotation_sign = self.get_direction_spin_face_wall(self.centering_threshold)
        self.get_logger().info(f"Rotation sign: {rotation_sign}", throttle_duration_sec=0.4)

        # If we are facing the wall, we change the state to TURN_AROUND
        if rotation_sign == 0:
            self.stop_movement()
            self.previous_pose = self.pose
            self.state = 'TURN_AROUND'
            return

        # Else we spin to face the wall ( get_spin_face_wall returns -1 or 1 based on the direction we should spin )
        self.move(speed=0.0, rads=rotation_sign * 0.1)
        

    def turn_around_behaviour(self):
        """ Turn 180 degrees around using previous pose"""

        """# If the back sensors are not similar or any of them is -1.0, we are not facing away from the wall
        if -1.0 in [self.bprox1, self.bprox2] or abs(self.bprox1 - self.bprox2) > self.turn_around_threshold:
            self.get_logger().info("Turning around...", throttle_duration_sec=0.4)
            self.move(speed=0.0, rads=0.3)
            return"""
        
        # Calculate the remaining yaw to spin 180 degrees ( we use the previous pose to calculate the remaining yaw )
        remaining_yaw = abs(self.pose[2] - self.previous_pose[2])

        # If the remaining yaw is greater than 3.1415, we are turning the wrong way, we should turn the other way
        if remaining_yaw > 3.1415:
            remaining_yaw = 2 * 3.1415 - remaining_yaw

        self.get_logger().info(f"Remaining Yaw: {remaining_yaw}", throttle_duration_sec=0.4)

        # If we haven't turned 180 degrees, we keep turning
        if remaining_yaw < 3.1415 - self.turn_around_threshold:
            self.move(speed=0.0, rads=0.2)
            return
        
        # If we are facing away from the wall, we change the state to FORWARD_2M
        self.stop_movement()
        self.previous_pose = self.pose
        self.state = 'FORWARD_2M'

    
    def forward_2m_behaviour(self):
        """ Move forward 2 meters """

        # Calculate ecuclidean distance between the current pose and the previous pose
        distance = self.euclidean_distance(self.previous_pose, self.pose)
        self.get_logger().info(f"Distance: {distance}")

        # Check if we have moved 2 meters, if not move forward, else change state to FORWARD
        if distance <= self.travel_distance:
            self.move(speed=0.5, rads=0.0)
            return

        self.stop_movement()
        self.get_logger().info("WE'RE DONE HERE!")
        exit(0)

    # --- Callbacks ---
    def odom_callback(self, msg):
        """ Callback for the odometry topic, here we get a 3d pose, and we convert it to a 2d pose (x, y, theta)"""
        pose2d = self.pose3d_to_2d(msg.pose.pose)
        self.pose = pose2d
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.4
        )

    def prox_callback(self, msg, prox):
        """ Callback for the proximity sensors, we store the value of the sensor """

        measure = round(msg.range, 5)

        if prox == 'prox1':
            self.prox1 = measure
        elif prox == 'prox2':
            self.prox2 = measure
        elif prox == 'prox3':
            self.prox3 = measure
        elif prox == 'prox4':
            self.prox4 = measure
        elif prox == 'bprox1':
            self.bprox1 = measure
        elif prox == 'bprox2':
            self.bprox2 = measure
        
        self.get_logger().info(f"Proximities: F:[{self.prox1}, {self.prox2}, {self.prox3}, {self.prox4}], diff: {abs(self.prox2 - self.prox3)} | B:[{self.bprox1}, {self.bprox2}]", throttle_duration_sec=0.1)


    # --- Movements Functions ---
    def move(self, speed=0.0, rads=0.0):
        """ Move the robot with a given speed and turn """
        velocity = Twist()
        
        # Forward Speed
        velocity.linear.x = float(speed)

        # Rotate speed
        velocity.angular.z = float(rads)

        self.velocity_publisher.publish(velocity)

    def stop_movement(self):
        """ Stop the robot """
        self.move()


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

    def get_direction_spin_face_wall(self, treshold):
        """ Return in which direction should the robot spin to face the wall ( make the prox2 and prox3 as similar as possible based on a treshold ) """
        
        # If the difference between the two central sensors is less than the treshold, we are facing the wall
        if self.prox2 != -1.0 and self.prox3 != -1.0 and abs(self.prox2 - self.prox3) <= treshold:
            self.get_logger().info("Facing the wall!!", throttle_duration_sec=0.4)
            return 0
        
        # get readings from the left and right sensors, ignore if one of them is -1.0
        left_prox = [prox for prox in [self.prox1, self.prox2] if prox != -1.0]
        right_prox = [prox for prox in [self.prox3, self.prox4] if prox != -1.0]

        # average the sum over the amount of correct readings, if no correct readings, set average to 10
        left_prox = sum(left_prox) / len(left_prox) if left_prox else 99
        right_prox = sum(right_prox) / len(right_prox) if right_prox else 99
        
        # Return the direction in which we should spin
        if left_prox < right_prox:
            return 1
        if left_prox > right_prox:
            return -1

        
        # If we reach this point, we were not close enough to the wall to take a good reading, we shouldn't be able to reach this point tho, setup proper tresholds!
        self.get_logger().error(f"BAD ENDING: [{self.prox1}, {self.prox2}, {self.prox3}, {self.prox4}].")
        self.stop_movement()
        raise Exception(" NOT CLOSE ENOUGH TO TAKE GOOD READING!! >:c")
    
    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - current_pose[0]), 2) +
                    pow((goal_pose[1] - current_pose[1]), 2))
        

def main(args=None):
    rclpy.init(args=args)
    
    # movemments based on states
    node = WallController()

    rclpy.spin(node)

    # Detect ctrl+c and stop the robot
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
