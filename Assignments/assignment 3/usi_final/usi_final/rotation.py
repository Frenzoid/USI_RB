import rclpy
from rclpy.node import Node
import numpy as np
import mediapipe as mp
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Int32
from robomaster_msgs.msg import GimbalCommand
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

"""
Author: Elvi Mihai Sabau Sabau
"""

class PrimaryController(Node):
    """ Get camera and publish finger number """
    def __init__(self):
        super().__init__('Primary_controllers')

        self.dir = "left"

        # Create cmd_gimbal publisher
        self.gimbal_publisher = self.create_publisher(GimbalCommand, '/yassine/cmd_gimbal', 10)

        # Create cmd_gimbal subscriber
        self.gimbal_subscriber = self.create_subscription(JointState, '/yassine/joint_states_p', self.gimbal_callback, 10)

        # Move the gimbal
        self.move_gimbal()
        self.create_timer(10.0, self.move_gimbal)

    def gimbal_callback(self, msg):
        """ Fucked up form of gimbal tracking:
            in msg we have 2 important arrays
                name: array of names of joints of the robot
                position: array of position ( in rads ) of each joint
        """


        # Default position if not found.
        name_pos = -1

        # Find on which position of the msg.name array is equals to position'yassine/gimbal_joint'
        for i in range(len(msg.name)):
            
            # Find the name in the array.
            if msg.name[i] == 'yassine/gimbal_joint':

                # Save position and break loop.
                name_pos = i
                break
        
        # If we didn't find the name, dont do anything ( it can happend )
        if name_pos == -1:
            return
        
        # Print the position of the joint ( yaw of gimbal )
        self.get_logger().info('gimbal_joint yaw position (rads): {0}'.format(msg.position[name_pos]))


    def move_gimbal(self):
        gimbal_command = GimbalCommand()
        gimbal_command.pitch_speed = 0.0

        if self.dir == "left":
            self.get_logger().info('Moving gimbal left O.o')
            gimbal_command.yaw_speed = 1.0
            self.dir = "right"
        elif self.dir == "right":
            self.get_logger().info('Moving gimbal right o.O')
            gimbal_command.yaw_speed = -1.0
            self.dir = "left"

        self.gimbal_publisher.publish(gimbal_command)

# --- Main ---
def main(args=None):
    rclpy.init(args=args)
    node = PrimaryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
