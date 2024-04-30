import cv2
import mediapipe as mp

# Initialize MediaPipe pose solution
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils  # Utility to draw landmarks

# Initialize video capture
cap = cv2.VideoCapture(0)

def classify_gesture(landmarks, width):
    # Extract necessary landmarks
    left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
    right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
    left_wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value]
    right_wrist = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value]
    
    # Individual hand states
    left_hand_state = "In" if left_wrist.x < left_shoulder.x else "Out"
    right_hand_state = "In" if right_wrist.x > right_shoulder.x else "Out"
    
    # Vertical position of hands
    left_vertical = "Up" if left_wrist.y < left_shoulder.y else "Down"
    right_vertical = "Up" if right_wrist.y < right_shoulder.y else "Down"
    
    return (left_hand_state, right_hand_state, left_vertical, right_vertical)

def main():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(frame)
        
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        if results.pose_landmarks:
            # Draw the pose annotation on the frame.
            mp_drawing.draw_landmarks(
                frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
            )
            landmarks = results.pose_landmarks.landmark
            left_state, right_state, left_vertical, right_vertical = classify_gesture(landmarks, frame.shape[1])
            cv2.putText(frame, f'Left Hand: {left_state}, {left_vertical}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f'Right Hand: {right_state}, {right_vertical}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Pose', frame)
        if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to exit
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
