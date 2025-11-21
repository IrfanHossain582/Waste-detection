from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
import numpy as np
import cv2
from time import sleep
from RPLCD.i2c import CharLCD
import RPi.GPIO as GPIO
import time

# Disable GPIO warnings
GPIO.setwarnings(False)

# Paths and class labels
model_path = '/home/pi/garbage_classification_CNN_ResNet_model.h5'
class_labels = ['cardboard', 'metal', 'paper', 'plastic']

# Load the pre-trained model
print(f"Loading model from: {model_path}")
try:
    model = load_model(model_path)
    print("Model loaded successfully!")
except Exception as e:
    print(f"Error loading model: {e}")
    exit()

# Initialize the LCD
try:
    lcd = CharLCD('PCF8574', 0x27)
    lcd.clear()
    lcd.write_string("System Ready!")
except Exception as e:
    print(f"Error initializing LCD: {e}")
    exit()

# GPIO setup for ultrasonic sensor
TRIG_PIN = 23  # Trigger pin
ECHO_PIN = 24  # Echo pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# GPIO setup for servo motor
SERVO_PIN = 18
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz frequency
servo_pwm.start(0)

# GPIO setup for DC motors
MOTOR1_PIN1 = 17  # IN1 for Motor A -> Physical Pin 11
MOTOR1_PIN2 = 27  # IN2 for Motor A -> Physical Pin 13
ENABLE_PIN = 25   # ENA for Motor A -> PWM speed control

MOTOR2_PIN1 = 5   # IN1 for Motor B -> Physical Pin 29
MOTOR2_PIN2 = 6   # IN2 for Motor B -> Physical Pin 31
ENB_PIN = 22      # ENB for Motor B -> PWM speed control

GPIO.setup([MOTOR1_PIN1, MOTOR1_PIN2, ENABLE_PIN, MOTOR2_PIN1, MOTOR2_PIN2, ENB_PIN], GPIO.OUT)

# PWM setup for motor speed control
motor_pwm = GPIO.PWM(ENABLE_PIN, 100)  # Frequency = 100 Hz
motor_pwm.start(8)  # Start with 13% duty cycle

motor_pwm1 = GPIO.PWM(ENB_PIN, 100)
motor_pwm1.start(8)

# Functions for controlling hardware
def run_motor(duty_cycle=8):
    # Motor A (MOTOR1_PIN1, MOTOR1_PIN2)
    GPIO.output(MOTOR1_PIN2, GPIO.LOW)  # Set direction (same for both motors)
    GPIO.output(MOTOR1_PIN1, GPIO.HIGH)

    # Motor B (MOTOR2_PIN1, MOTOR2_PIN2)
    GPIO.output(MOTOR2_PIN2, GPIO.LOW)  # Set direction
    GPIO.output(MOTOR2_PIN1, GPIO.HIGH)

    # Set the same speed for both motors using PWM
    motor_pwm.ChangeDutyCycle(duty_cycle)
    motor_pwm1.ChangeDutyCycle(duty_cycle)

    print(f"Both motors running at {duty_cycle}% duty cycle in the same direction.")

def stop_motor():
    # Stop Motor A
    GPIO.output([MOTOR1_PIN1, MOTOR1_PIN2], GPIO.LOW)
    motor_pwm.ChangeDutyCycle(0)

    # Stop Motor B
    GPIO.output([MOTOR2_PIN1, MOTOR2_PIN2], GPIO.LOW)
    motor_pwm1.ChangeDutyCycle(0)

    print("Both motors stopped.")

def rotate_servo(angle):
    duty_cycle = 2.5 + (angle / 18)  # Map angle to duty cycle
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1)  # Allow time for servo to move
    servo_pwm.ChangeDutyCycle(0)  # Avoid jitter

def measure_distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2
    return distance

def display_on_lcd(line1, line2=""):
    try:
        lcd.clear()
        lcd.cursor_pos = (0, 0)
        lcd.write_string(line1)
        if line2:
            lcd.cursor_pos = (1, 0)
            lcd.write_string(line2)
    except Exception as e:
        print(f"Error displaying on LCD: {e}")

def preprocess_and_predict(image):
    try:
        image = cv2.resize(image, (224, 224))
        image = img_to_array(image)
        image = np.expand_dims(image, axis=0)
        prediction = model.predict(image, verbose=0)
        predicted_class = np.argmax(prediction)
        predicted_label = class_labels[predicted_class]
        confidence = prediction[0][predicted_class]
        return predicted_label, confidence
    except Exception as e:
        print(f"Error during prediction: {e}")
        return None, None

def initialize_camera():
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Could not open camera.")
        exit()
    return camera

# Main loop
camera = initialize_camera()
CONFIDENCE_THRESHOLD = 0.5

try:
    print("System is running. Press Ctrl+C to stop.")
    run_motor()  # Start the motors

    while True:
        distance = measure_distance()
        print(f"Measured Distance: {distance:.2f} cm")

        if 0 < distance <= 12:  # Object detected
            print("Object detected.")
            stop_motor()
            time.sleep(2)

            # Capture and classify image
            ret, frame = camera.read()
            if not ret:
                print("Camera frame capture failed.")
                continue

            predicted_label, confidence = preprocess_and_predict(frame)
            if predicted_label and confidence >= CONFIDENCE_THRESHOLD:
                print(f"Prediction: {predicted_label}, Confidence: {confidence:.2f}")
                display_on_lcd("Detected:", predicted_label)

                if predicted_label in class_labels:
                    print("Recyclable object detected.")
                    rotate_servo(180)
                    time.sleep(2)
                    rotate_servo(0)
                else:
                    print("Non-recyclable object detected.")

            else:
                display_on_lcd("Low confidence.", "Not Recyclable")
                print("Prediction confidence too low.")

            print("Restarting motors.")
            run_motor()
        else:
            display_on_lcd("No object", "Detected")
            time.sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram interrupted. Exiting...")

finally:
    camera.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    lcd.clear()
    servo_pwm.stop()
    motor_pwm.stop()
    motor_pwm1.stop()
    print("Resources cleaned up.")
