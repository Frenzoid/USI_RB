import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from robomaster_msgs.msg import GimbalCommand

#from robomaster_msgs.action import MoveGimbal_FeedbackMessage 


"""
Author: Elvi Mihai Sabau Sabau
"""

class PrimaryController(Node):
    """ Get camera and publish finger number """
    def __init__(self):
        super().__init__('Primary_controllers')

        self.state_human = 0
        self.gimball_rads = 0.0
        self.direction= 0.5
        # Initialize MediaPipe pose solution
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()
        self.mp_drawing = mp.solutions.drawing_utils  # Utility to draw landmarks
        #create a subscriber
        self.image_sub = self.create_subscription(Image, '/camera/image_color', self.image_callback, 10)
        self.image_sub  # prevent unused variable warning
        self.bridge = CvBridge()
        #create a publisher 
        self.finger_pub = self.create_publisher(GimbalCommand, '/cmd_gimbal', 10)
        #create a subscriber for the feedback
        self.feedback_sub = self.create_subscription(JointState, '/joint_states_p', self.gimbal_call, 10)
        
        
    def gimbal_call(self, msg):
        
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
            if msg.name[i] == 'gimbal_joint':

                # Save position and break loop.
                name_pos = i
                break
        
        # If we didn't find the name, dont do anything ( it can happend )
        if name_pos == -1:
            return
        
        # Print the position of the joint ( yaw of gimbal )
        self.get_logger().info('gimbal_joint yaw position (rads): {0}'.format(msg.position[name_pos]))
        self.gimball_rads = msg.position[name_pos]
        
 
    def serch_human(self):
        
    
        if self.gimball_rads <-4.2:

            self.direction = -0.5
        elif self.gimball_rads >4.2:
            self.direction = 0.5
        
        self.move(self.direction, 0.0)


    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg)
        # Process image
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(frame)
        if results.pose_landmarks:
            self.state_human = 1
            # Draw the pose annotation on the frame.
            #self.mp_drawing.draw_landmarks(
            #    frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS,
            #    self.mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
            #    self.mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
            #)
            landmarks = results.pose_landmarks.landmark
            #left_state, right_state, left_vertical, right_vertical = classify_gesture(landmarks, frame.shape[1])
            
            vertical_position, horisental_position = self.get_values(landmarks, frame.shape[1])
            #cv2.putText(frame, f'Human position x : {vertical_position}', (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 0), 2)
            #cv2.putText(frame, f'Human position y:  {horisental_position}', (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 0), 2)
            
            vertical, horisental = self.get_position(vertical_position, horisental_position,0.1)
            if vertical_position>0.9:
                self.state_human = 0
                self.serch_human()
                return
            
            hmove = None
            vmove = None
            
            if self.state_human == 1:
                # Move the vertical  
                if vertical == "up" :
                    vmove = -1.0
                elif vertical == "down" :
                    vmove = 1.0
                else:
                    vmove = 0.0
                #move horizontal
                if horisental == "right":
                    hmove = 1.0
                elif horisental == "left":
                    hmove = -1.0
                else:
                    hmove = 0.0

                self.move(hmove, vmove)


            #cv2.putText(frame, f'vertical pos : {vertical}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 0), 2)
            #cv2.putText(frame, f'horizontal pos: {horisental}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 0), 2)
            
            #printing is screen if the shoulders are in the center of the screen or not with a epsilon error

        else:
            cv2.putText(frame, 'No pose detected', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            self.state_human = 0
            #serch fore human 
            self.serch_human()



        #cv2.imshow('Pose', frame)
        #if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to exit
        #    return
    
        
    
    def get_values(self,landmarks, width):
        # Extract necessary landmarks
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value]

        # Individual hand states
        horisental_position = round((left_shoulder.x  + right_shoulder.x)/2,2)
        vertical_position =round((left_shoulder.y  + right_shoulder.y)/2,2)
    
        return (vertical_position, horisental_position)

    def get_position(self,vertical_position, horisental_position,epsilon):

        if epsilon > abs(vertical_position -0.5) :
            vertical = "center"
        elif vertical_position -0.5 > 0:
            vertical = "down"
        else:
            vertical = "up"
            
        if epsilon > abs(horisental_position -0.5) :
            horisental = "center"
        elif horisental_position -0.5 > 0:
            horisental = "right"
        else:
            horisental = "left"

        return (vertical, horisental)
    
    
    
    def move(self, yaw=0.0, pitch=0.0):
        #self.get_logger().info(f"Moving gimbal: yaw={yaw}, pitch={pitch}")
        msg = GimbalCommand()
        msg.yaw_speed = yaw
        msg.pitch_speed = pitch
        self.finger_pub.publish(msg)

    
# --- Main ---
def main(args=None):
    rclpy.init(args=args)
    node = PrimaryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()