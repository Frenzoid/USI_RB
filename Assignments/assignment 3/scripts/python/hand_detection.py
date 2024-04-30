import cv2
import mediapipe as mp

# Initializing mediapipe hands model
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Function to count fingers up
def count_fingers_up(hand_landmarks, image_shape):
    finger_tips_ids = [4, 8, 12, 16, 20]
    up_count = 0
    for idx in finger_tips_ids:
        finger_tip_y = hand_landmarks.landmark[idx].y * image_shape[0]
        finger_bottom_y = hand_landmarks.landmark[idx - 2].y * image_shape[0]
        if finger_tip_y < finger_bottom_y:
            up_count += 1
    return up_count

# Capture video from webcam
cap = cv2.VideoCapture(0)

# Draw landmarks and count fingers
mp_drawing = mp.solutions.drawing_utils
while cap.isOpened():
    success, image = cap.read()
    if not success:
        continue

    # Flip image for selfie-view and convert to RGB
    image = cv2.flip(image, 1)
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_image)

    # Draw hand landmarks and count fingers
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # If its first hand print the count on the top right corner
            if results.multi_hand_landmarks.index(hand_landmarks) == 0:
                fingers_up = count_fingers_up(hand_landmarks, image.shape)
                fingers_up_count = count_fingers_up(hand_landmarks, image.shape)
                cv2.putText(image, f'Fingers Up: {fingers_up_count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            # Else on the left corner
            else:
                fingers_up = count_fingers_up(hand_landmarks, image.shape)
                cv2.putText(image, f'Fingers Up: {fingers_up}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                
    # Display the image
    cv2.imshow('Live Hand Tracking', image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

# Release resources
cap.release()
cv2.destroyAllWindows()