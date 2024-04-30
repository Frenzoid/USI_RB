import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

"""
Author: Elvi Mihai Sabau Sabau
"""

class PrimaryController(Node):
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
        self.create_subscription(Image, '/robomaster/camera/image_color', self.image_callback, 10)

    
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

            # Classify the gesture
            landmarks = results.pose_landmarks.landmark
            vertical, horizontal = self.classify_gesture(landmarks, frame.shape[1])

            # Display the gesture on the frame
            cv2.putText(frame, f'Vertical: {vertical}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, f'Horizontal: {horizontal}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            #  Display the frame
            cv2.imshow('Frame', frame)
            cv2.waitKey(1)
        


    # Function to count fingers up
    def classify_gesture(self, landmarks, width):

        # Extract necessary landmarks
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
        left_wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value]
        right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value]
        
        # Shoulder and wrist coordinates
        shoulders_y = (left_shoulder.y + right_shoulder.y) / 2
        wrists_y = (left_wrist.y + right_wrist.y) / 2
        shoulders_x_distance = abs(right_shoulder.x - left_shoulder.x)
        wrists_x_distance = abs(right_wrist.x - left_wrist.x)
        
        # Gesture detection logic
        if wrists_y < shoulders_y:
            vertical = "Hands Up"
        else:
            vertical = "Hands Down"
        
        if wrists_x_distance > shoulders_x_distance:
            horizontal = "Hands Separated"
        else:
            horizontal = "Hands Together"
        
        return vertical, horizontal
       
    
# --- Main ---
def main(args=None):
    rclpy.init(args=args)
    node = PrimaryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
