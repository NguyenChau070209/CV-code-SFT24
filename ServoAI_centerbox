import argparse
import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from utils import visualize
from picamera2 import Picamera2
import RPi.GPIO as GPIO
from time import sleep

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()

# Initialize the camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Servo setup
servo_pin = 18  # GPIO pin connected to servo
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# LED setup
led_pin = 15  # GPIO pin connected to LED
GPIO.setup(led_pin, GPIO.OUT)

# Initialize PWM for servo (50Hz)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

# Variables to track servo angle direction
servo_angle = 90  # Start at the middle angle
servo_direction = 1  # 1 for increasing, -1 for decreasing

def set_angle(angle):
    """Set the servo angle."""
    angle = max(0, min(180, angle))  # Ensure the angle is within 0 to 180 degrees
    duty = angle / 18 + 2
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    sleep(0.1)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

def move_servo_continuous():
    """Continuously move the servo back and forth."""
    global servo_angle, servo_direction
    set_angle(servo_angle)
    servo_angle += 10 * servo_direction
    if servo_angle >= 180 or servo_angle <= 0:
        servo_direction *= -1

def move_servo_to_center(bbox_center_x, bbox_center_y, frame_center_x, frame_center_y):
    """Move servo to center detected object in the frame."""
    tolerance = 20  # Adjust if necessary
    if abs(bbox_center_x - frame_center_x) > tolerance:
        if bbox_center_x < frame_center_x:
            print("Moving servo left.")
            set_angle(servo_angle - 5)
        else:
            print("Moving servo right.")
            set_angle(servo_angle + 5)
    
    if abs(bbox_center_y - frame_center_y) > tolerance:
        if bbox_center_y < frame_center_y:
            print("Moving servo up.")
            set_angle(servo_angle + 5)
        else:
            print("Moving servo down.")
            set_angle(servo_angle - 5)

def run(model: str, max_results: int, score_threshold: float, 
        camera_id: int, frame_width: int, frame_height: int) -> None:
    
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2
    detection_result_list = []

    def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
        detection_result_list.append(result)

    # Initialize the object detection model
    base_options = python.BaseOptions(model_asset_path=model)
    options = vision.ObjectDetectorOptions(base_options=base_options,
                                           running_mode=vision.RunningMode.LIVE_STREAM,
                                           max_results=max_results, score_threshold=score_threshold,
                                           result_callback=save_result)
    detector = vision.ObjectDetector.create_from_options(options)

    while True:
        im = picam2.capture_array()
        image = cv2.resize(im, (frame_width, frame_height))
        image = cv2.flip(image, -1)
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
        detector.detect_async(mp_image, time.time_ns() // 1_000_000)

        if detection_result_list:
            # If fire is detected, stop servo and turn on LED
            current_frame = visualize(image, detection_result_list[0])

            for detection in detection_result_list[0].detections:
                bbox = detection.bounding_box
                x_min = bbox.origin_x
                y_min = bbox.origin_y
                width = bbox.width
                height = bbox.height
                bbox_center_x = x_min + width // 2
                bbox_center_y = y_min + height // 2

                print(f"Fire detected! Center coordinates: ({bbox_center_x}, {bbox_center_y}), "
                      f"Width: {width}, Height: {height}")

                # Move servo to center detected box
                move_servo_to_center(bbox_center_x, bbox_center_y, frame_center_x, frame_center_y)

            pwm.stop()  # Stop servo after adjustment
            GPIO.output(led_pin, GPIO.HIGH)  # Turn on LED when fire is detected
            detection_result_list.clear()

        else:
            # If no fire is detected, continue sweeping servo and turn off LED
            GPIO.output(led_pin, GPIO.LOW)  # Turn off LED when no fire is detected
            move_servo_continuous()

        # Display the camera frame
        cv2.imshow('object_detection', image)
        
        if cv2.waitKey(1) == 27:  # Press ESC to exit
            break

    detector.close()
    pwm.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--model', help='Path of the object detection model.', default='best.tflite')
    parser.add_argument('--maxResults', help='Max number of detection results.', type=int, default=5)
    parser.add_argument('--scoreThreshold', help='The score threshold of detection results.', type=float, default=0.6)
    parser.add_argument('--cameraId', help='Id of camera.', type=int, default=0)
    parser.add_argument('--frameWidth', help='Width of frame to capture from camera.', type=int, default=640)
    parser.add_argument('--frameHeight', help='Height of frame to capture from camera.', type=int, default=480)
    args = parser.parse_args()

    run(args.model, args.maxResults, args.scoreThreshold, args.cameraId, args.frameWidth, args.frameHeight)

if __name__ == '__main__':
    main()
