import rclpy
import tf_transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.node import Node
from nav_msgs.msg import Odometry

"""
Author: Elvi Mihai Sabau Sabau
"""

class PrimaryController(Node):
    """ Walk until an obstacle is detected, then turn towards the direction where there is no obstacles"""

    def __init__(self):
        super().__init__('walk_controller')

        #  --- Varialbes ---
        # Control loop hz
        self.control_loop_hz = 0.1

        # Initialize proximity sensors
        self.prox_sensors = {'prox1': -1.0, 'prox2': -1.0, 'prox3': -1.0, 'prox4': -1.0}

        # Initialize the robot's pose
        self.pose = None
        
        # Speed vel and turn
        self.speed = 0.5
        self.turn = 1.0

        # Threshold of how close should the robot be to an obstacle to consider it detected
        self.threshold = 0.05


        # --- Publishers and Subscribers ---
        # Create a publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to get proximity sensor readings
        self.prox1_subscriber = self.create_subscription(Range, 'proximity/left', lambda msg: self.prox_callback(msg, 'prox1'), 10)
        self.prox2_subscriber = self.create_subscription(Range, 'proximity/center_left', lambda msg: self.prox_callback(msg, 'prox2'), 10)
        self.prox3_subscriber = self.create_subscription(Range, 'proximity/center_right', lambda msg: self.prox_callback(msg, 'prox3'), 10)
        self.prox4_subscriber = self.create_subscription(Range, 'proximity/right', lambda msg: self.prox_callback(msg, 'prox4'), 10)

        # Get Odometry data to get the robot's position
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', lambda msg: self.odom_callback(msg), 10)


        # --- Timers ---
        # Control loop timer
        self.create_timer(self.control_loop_hz, self.control_loop)


    # --- Control Loop ---
    def control_loop(self):
        """Control loop to move the robot based on proximity sensor readings"""

        if self.pose is None:
            self.get_logger().warn("Waiting for odometry data...")
            return

        # Get sensor readings
        prox1, prox2, prox3, prox4 = self.prox_sensors['prox1'], self.prox_sensors['prox2'], self.prox_sensors['prox3'], self.prox_sensors['prox4']

        # Get the maximum proximity sensors from each side
        prox_left = max(prox1, prox2) 
        prox_right = max(prox3, prox4)

        # Move forward if no obstacles detected
        if all(sensor == -1.0 for sensor in [prox_left, prox_right]):
            self.uncertain = False
            self.get_logger().info("No obstacles detected. Moving forward.")
            self.move(self.speed, 0.0) # Move forward
            return

        # Obstacle on the left or center-left
        if prox_right == -1.0 and prox_left < self.threshold:
            self.uncertain = False
            self.get_logger().info("Obstacle detected on the left. Turning right.")
            self.move(0, -self.turn)  # Turn right
            return

        # Obstacle on the right or center-right
        if prox_left == -1.0 and prox_right < self.threshold:
            self.uncertain = False
            self.get_logger().info("Obstacle detected on the right. Turning left.")
            self.move(0, self.turn)  # Turn left
            return

        # In any other case if we detect something but were not sure on which side its closer, turn right
        self.move(0, -self.turn)


    # --- Callbacks ---
    # Proximity sensor callback
    def prox_callback(self, msg, sensor_id):
        """Callback function for proximity sensors"""
        self.prox_sensors[sensor_id] = round(msg.range, 4)
        self.get_logger().info(f"Proximity sensors: {self.prox_sensors}")

    # Odometry callback
    def odom_callback(self, msg):
        """ Callback for the odometry topic, here we get a 3d pose, and we convert it to a 2d pose (x, y, theta)"""
        pose2d = self.pose3d_to_2d(msg.pose.pose)
        self.pose = pose2d
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.4
        )


    # --- Helper Functions ---
    # Move the robot
    def move(self, speed, turn):
        twist = Twist()
        twist.linear.x = float(speed)
        twist.angular.z = float(turn)
        self.velocity_publisher.publish(twist)

    # Convert a 3D pose to a 2D pose
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
    
# --- Main ---
def main(args=None):
    rclpy.init(args=args)
    node = PrimaryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
