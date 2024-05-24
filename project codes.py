//code for Car//
#include <IRremote.h>
#include <Servo.h>

// IR receiver pin
const int IR_PIN = 11;

// Motor pins for the car
const int motor1Pin1 = 2; // Motor 1, pin 1
const int motor1Pin2 = 3; // Motor 1, pin 2
const int motor2Pin1 = 4; // Motor 2, pin 1
const int motor2Pin2 = 5; // Motor 2, pin 2
const int motor3Pin1 = 6; // Motor 3, pin 1
const int motor3Pin2 = 7; // Motor 3, pin 2
const int motor4Pin1 = 8; // Motor 4, pin 1
const int motor4Pin2 = 9; // Motor 4, pin 2

// Servo pins for the arm
const int servoBasePin = 10;
const int servoArmPin = 12;
const int servoGripperPin = 13;

// Define IRremote object
IRrecv irrecv(IR_PIN);
decode_results results;

// Define servo objects
Servo servoBase;
Servo servoArm;
Servo servoGripper;

void setup() {
  // Define motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);

  // Attach servo objects to their pins
  servoBase.attach(servoBasePin);
  servoArm.attach(servoArmPin);
  servoGripper.attach(servoGripperPin);

  // Start the IR receiver
  irrecv.enableIRIn();
}

void loop() {
  if (irrecv.decode(&results)) {
    switch (results.value) {
      case 0xFF30CF: // Forward
        moveForward();
        break;
      case 0xFF18E7: // Backward
        moveBackward();
        break;
      case 0xFF10EF: // Left
        turnLeft();
        break;
      case 0xFF38C7: // Right
        turnRight();
        break;
      case 0xFFA25D: // Stop
        stopMotors();
        break;
      case 0xFF02FD: // Move arm up
        moveArmUp();
        break;
      case 0xFFC23D: // Move arm down
        moveArmDown();
        break;
      case 0xFFE01F: // Open gripper
        openGripper();
        break;
      case 0xFFA857: // Close gripper
        closeGripper();
        break;
      default:
        break;
    }
    irrecv.resume();
  }
}

// Car movement functions
void moveForward() {
  // Write motor configurations to move forward
}

void moveBackward() {
  // Write motor configurations to move backward
}

void turnLeft() {
  // Write motor configurations to turn left
}

void turnRight() {
  // Write motor configurations to turn right
}

void stopMotors() {
  // Write motor configurations to stop
}

// Arm control functions
void moveArmUp() {
  // Move the arm up by adjusting servo positions
}

void moveArmDown() {
  // Move the arm down by adjusting servo positions
}

void openGripper() {
  // Open the gripper by adjusting servo position
}

void closeGripper() {
  // Close the gripper by adjusting servo position
}
#####code for Human Detection Model#####

import cv2
import imutils
import numpy as np
import RPi.GPIO as GPIO
import time

# Set up GPIO pin for the buzzer
BUZZER_PIN = 18  # Change to the correct pin number

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.output(BUZZER_PIN, GPIO.LOW)

protopath = "model/object_detection/MobileNetSSD_deploy.prototxt"
modelpath = "model/object_detection/MobileNetSSD_deploy.caffemodel"
detector = cv2.dnn.readNetFromCaffe(protopath, modelpath)

CLASSES = ["person"]

cap = cv2.VideoCapture(0)  # Use the first connected camera

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = imutils.resize(frame, width=600)
    (H, W) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (W, H), 127.5)
    detector.setInput(blob)
    person_detections = detector.forward()

    person_detected = False

    for i in np.arange(0, person_detections.shape[2]):
        confidence = person_detections[0, 0, i, 2]
        if confidence > 0.5:
            idx = int(person_detections[0, 0, i, 1])
            if CLASSES[idx] != "person":
                continue

            person_detected = True
            person_box = person_detections[0, 0, i, 3:7] * np.array([W, H, W, H])
            (startX, startY, endX, endY) = person_box.astype("int")
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

    if person_detected:
        GPIO.output(BUZZER_PIN, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(BUZZER_PIN, GPIO.LOW)

    cv2.imshow("Application", frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
