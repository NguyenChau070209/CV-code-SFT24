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
led_pin = 17  # GPIO pin connected to LED
GPIO.setup(led_pin, GPIO.OUT)

# Initialize PWM for servo (50Hz)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    """Set the servo angle."""
    duty = angle / 18 + 2
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    sleep(0.5)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

def run(model: str, max_results: int, score_threshold: float, 
        camera_id: int, width: int, height: int) -> None:
    """Continuously run inference on images acquired from the camera."""
    
    # Visualization parameters
    row_size = 50  # pixels
    left_margin = 24  # pixels
    text_color = (0, 0, 0)  # black
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 10
    detection_frame = None
    detection_result_list = []

    def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
        """Callback to process detection results."""
        global FPS, COUNTER, START_TIME

        # Calculate the FPS
        if COUNTER % fps_avg_frame_count == 0:
            FPS = fps_avg_frame_count / (time.time() - START_TIME)
            START_TIME = time.time()

        detection_result_list.append(result)
        COUNTER += 1

    # Initialize the object detection model
    base_options = python.BaseOptions(model_asset_path=model)
    options = vision.ObjectDetectorOptions(base_options=base_options,
                                           running_mode=vision.RunningMode.LIVE_STREAM,
                                           max_results=max_results, score_threshold=score_threshold,
                                           result_callback=save_result)
    detector = vision.ObjectDetector.create_from_options(options)

    # Continuously capture images from the camera and run inference
    while True:
        im = picam2.capture_array()
        image = cv2.resize(im, (640, 480))
        image = cv2.flip(image, -1)

        # Convert the image from BGR to RGB as required by the TFLite model
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # Run object detection using the model
        detector.detect_async(mp_image, time.time_ns() // 1_000_000)

        # Show the FPS
        fps_text = 'FPS = {:.1f}'.format(FPS)
        text_location = (left_margin, row_size)
        current_frame = image.copy()  # Use a copy to avoid altering the original frame
        cv2.putText(current_frame, fps_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                    font_size, text_color, font_thickness, cv2.LINE_AA)

        if detection_result_list:
            # If fire is detected, stop servo and turn on LED
            current_frame = visualize(current_frame, detection_result_list[0])
            detection_frame = current_frame

            # Get the coordinates of the fire detection bounding box
            for detection in detection_result_list[0].detections:
                bbox = detection.bounding_box
                x_min = bbox.origin_x
                y_min = bbox.origin_y
                x_max = bbox.origin_x + bbox.width
                y_max = bbox.origin_y + bbox.height
                print(f"Fire detected! Coordinates: ({x_min}, {y_min}), ({x_max}, {y_max})")

            print("Fire detected! Stopping servo and turning on LED.")
            pwm.stop()  # Stop the servo when fire is detected
            GPIO.output(led_pin, GPIO.HIGH)  # Turn on LED
            detection_result_list.clear()
        else:
            # If no fire is detected, continue rotating servo and turn off LED
            GPIO.output(led_pin, GPIO.LOW)  # Turn off LED
            for angle in range(0, 181, 10):  # Sweep from 0 to 180 degrees
                set_angle(angle)
                sleep(0.1)  # Wait a bit for the servo to complete rotation

        if detection_frame is not None:
            cv2.imshow('object_detection', detection_frame)

        # Stop the program if the ESC key is pressed
        if cv2.waitKey(1) == 27:
            break

    detector.close()
    pwm.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--model',
        help='Path of the object detection model.',
        required=False,
        default='best.tflite')
    parser.add_argument(
        '--maxResults',
        help='Max number of detection results.',
        required=False,
        type=int,
        default=5)
    parser.add_argument(
        '--scoreThreshold',
        help='The score threshold of detection results.',
        required=False,
        type=float,
        default=0.6)
    parser.add_argument(
        '--cameraId',
        help='Id of camera.',
        required=False,
        type=int,
        default=0)
    parser.add_argument(
        '--frameWidth',
        help='Width of frame to capture from camera.',
        required=False,
        type=int,
        default=640)
    parser.add_argument(
        '--frameHeight',
        help='Height of frame to capture from camera.',
        required=False,
        type=int,
        default=480)
    args = parser.parse_args()

    run(args.model, int(args.maxResults),
        args.scoreThreshold, int(args.cameraId), args.frameWidth, args.frameHeight)

if __name__ == '__main__':
    main()
