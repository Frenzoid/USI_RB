import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

"""
Author: Elvi Mihai Sabau Sabau
"""

class Publish_Gestures(Node):
    """ Get camera and publish finger number """
    def __init__(self):
        super().__init__('Primary_controllers')

        # Initialize MediaPipe pose solution
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()
        self.mp_drawing = mp.solutions.drawing_utils  # Utility to draw landmarks

        # Initialize OpenCV bridge ( from ROS Docs )
        self.bridge = CvBridge()

        # Create image subscriber for /camera/image_color
        self.create_subscription(Image, '/camera/image_color', self.image_callback, 10) #/yassine/camera/image_color

        self.movdir_publisher = self.create_publisher(String, '/cmd_command', 10)

    
    def image_callback(self, msg):
        self.get_logger().info('Received an image.')
        
        # Convert the ROS image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg)

        # Process the frame
        results = self.pose.process(frame)

        # Draw the pose landmarks on the frame
        if results.pose_landmarks:

                        # Draw the pose annotation / landmarks on the frame.
            self.mp_drawing.draw_landmarks(
            frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS,
            self.mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
            self.mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
            )

            landmarks = results.pose_landmarks.landmark
            left_state, right_state, left_vertical, right_vertical = self.classify_gesture(landmarks, frame.shape[1])[0]
            visibilities = self.classify_gesture(landmarks, frame.shape[1])[1]
            #if any(visibility < 0.8 for visibility in visibilities): [3]
            if any((visibilities[0] < 0.8, visibilities[1] < 0.8, visibilities[2] < 0.2, visibilities[3] < 0.3, visibilities[4] < 0.3)):
                cv2.putText(frame, f'NOT FULL BODY VISIBILITY', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 7)
                self.get_logger().info(f'Bad visibilities: /n {visibilities}')
                #[0.9982137680053711, 0.9955745339393616, 0.554503321647644, 0.9246881008148193, 0.9246881008148193]

            
            #  Display the frame
                cv2.imshow('Frame', frame)
                cv2.waitKey(1)
            else:
                direction = 'UNKNOWN'  # Default direction
                movement = 'UNKNOWN'  # Default movement
                # For the left arm
                match (left_state, left_vertical):
                    case ('Out', 'Up'):
                        direction =  '1' #'LEFT'
                    case ('Out', 'Down'):
                        direction = '-1' #'RIGHT'
                    case ('In', _):  # The underscore (_) is a wildcard that matches anything
                        direction = '0' #'STOP'

                # For the right arm
                match (right_state, right_vertical):
                    case ('Out', 'Up'):
                        movement = '1' #'FORWARD'
                    case ('Out', 'Down'):
                        movement = '-1' #'BACKWARD'
                    case ('In', _):
                        movement = '0' #'STOP'

                cv2.putText(frame, f'Left Hand (DIRECTION): {direction}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 7)
                cv2.putText(frame, f'Right Hand (MOVEMENT): {movement}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 7)
                msg = String()
                msg.data = f'{direction},{movement}'
                self.movdir_publisher.publish(msg)
                

            #  Display the frame
                cv2.imshow('Frame', frame)
                cv2.waitKey(1)
        else:
            self.get_logger().info(f'no pose landmarks:')
            cv2.putText(frame, 'NO POSE LANDMARKS', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 7)
            cv2.imshow('Frame', frame)
            cv2.waitKey(1)


    # Function to count fingers up
    def classify_gesture(self, landmarks, width):

    # Extract necessary landmarks
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
        left_wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value]
        right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value]

        # Extract visibility values
        left_shoulder_visibility = left_shoulder.visibility
        right_shoulder_visibility = right_shoulder.visibility
        left_wrist_visibility = left_wrist.visibility
        right_wrist_visibility = right_wrist.visibility

        # Individual hand states
        left_hand_state = "In" if left_wrist.x < left_shoulder.x else "Out"
                            #IN                                        #OUT
        right_hand_state = "In" if right_wrist.x > right_shoulder.x else "Out"
                            # IN                                        #OUT
        # Vertical position of hands
        left_vertical = "Up" if left_wrist.y < left_shoulder.y else "Down"
        right_vertical = "Up" if right_wrist.y < right_shoulder.y else "Down"

        return ([left_hand_state, right_hand_state, left_vertical, right_vertical],[left_shoulder_visibility,right_shoulder_visibility, left_wrist_visibility, right_wrist_visibility, right_wrist.visibility])
        
    
# --- Main ---
def main(args=None):
    rclpy.init(args=args)
    node = Publish_Gestures()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()