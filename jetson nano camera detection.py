import cv2
import torch
import time
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('/home/khaled/best.pt')

# Use GPU if available
if torch.cuda.is_available():
    model.to('cuda')

# Initialize the camera
cap = cv2.VideoCapture(0)  # Change 0 to the correct camera ID if necessary

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set a smaller frame size for faster processing
frame_width = 640
frame_height = 480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

while True:
    start_time = time.time()
    
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break
    
    # Resize frame for faster processing
    resized_frame = cv2.resize(frame, (frame_width, frame_height))
    
    # Run YOLOv8 inference
    results = model(resized_frame)
    
    # Process results
    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = box.conf.item()
            cls = box.cls.item()
            label = f'{model.names[int(cls)]} {conf:.2f}'
            
            # Draw bounding boxes and labels on the frame
            cv2.rectangle(resized_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(resized_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Display the frame
    cv2.imshow('Camera Output', resized_frame)
    
    # Calculate elapsed time and wait to achieve 1 frame per second
    elapsed_time = time.time() - start_time
    time_to_wait = max(1.0 - elapsed_time, 0)
    if cv2.waitKey(int(time_to_wait * 1000)) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
