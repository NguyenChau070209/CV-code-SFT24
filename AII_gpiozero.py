import argparse
import sys
import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from utils import visualize
from picamera2 import Picamera2
from time import sleep
import threading
import asyncio
from telegram import Bot
from gpiozero import Servo, LED, OutputDevice

# ========== Cấu hình Telegram ========== 
TOKEN = 'YOUR_TOKEN'
CHAT_ID = 'YOUR_CHAT_ID'
bot = Bot(token=TOKEN)

telegram_sending = False
telegram_thread = None

def send_telegram_message():
    asyncio.run(send_message_periodically())

async def send_message_periodically():
    while telegram_sending:
        try:
            await bot.send_message(chat_id=CHAT_ID, text="Nhà bạn đang có cháy!!!")
            print("Đã gửi tin nhắn Telegram")
        except Exception as e:
            print(f"Lỗi khi gửi tin nhắn Telegram: {e}")
        await asyncio.sleep(2)

# ========== Cấu hình GPIO với GPIOzero ========== 
# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Servo setup for X and Y axes using GPIOzero
servo_x = Servo(18)
servo_y = Servo(14)

# LED setup using GPIOzero
led = LED(15)

# Relay setup using GPIOzero
relay = OutputDevice(17)

# Variables to track servo angle direction for both servos
servo_angle_x = 0
servo_angle_y = 0
servo_direction_x = 1
servo_direction_y = 1

servo_angle = 0
servo_direction = 1

# Function to display FPS
def show_fps(image):
    global COUNTER, FPS, START_TIME
    COUNTER += 1
    if (time.time() - START_TIME) > 1:
        FPS = COUNTER / (time.time() - START_TIME)
        COUNTER = 0
        START_TIME = time.time()
    
    fps_text = f'FPS = {FPS:.1f}'
    text_location = (10, 30)
    font_size = 1
    font_color = (255, 255, 255)
    font_thickness = 2
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                font_size, font_color, font_thickness, cv2.LINE_AA)

# Function to set servo angle for both X and Y axes using GPIOzero
def set_angle(servo, angle):
    position = (angle - 90) / 90  # Convert angle to range (-1, 1)
    servo.value = position
    sleep(0.08)

# Move servos based on detection box position
def move_servo_to_center(bbox_center_x, bbox_center_y, frame_center_x, frame_center_y):
    global servo_angle_x, servo_angle_y
    tolerance = 30

    # X-axis control
    if abs(bbox_center_x - frame_center_x) > tolerance:
        if bbox_center_x < frame_center_x:
            new_angle_x = max(servo_angle_x - 5, -90)
            set_angle(servo_x, new_angle_x)
            print("move to left")
        else:
            new_angle_x = min(servo_angle_x + 5, 90)
            set_angle(servo_x, new_angle_x)
            print("move to right")
        servo_angle_x = new_angle_x
    
    # Y-axis control
    if abs(bbox_center_y - frame_center_y) > tolerance:
        if bbox_center_y < frame_center_y:
            new_angle_y = min(servo_angle_y + 5, 90)
            set_angle(servo_y, new_angle_y)
            print("move up")
        else:
            new_angle_y = max(servo_angle_y - 5, -90)
            set_angle(servo_y, new_angle_y)
            print("move down")
        servo_angle_y = new_angle_y

# Move servo continuously between 0 and 180 degrees using GPIOzero
def move_servo_continuous():
    global servo_angle, servo_direction
    set_angle(servo_x, servo_angle)
    servo_angle += 10 * servo_direction
    if servo_angle >= 180 or servo_angle <= 0:
        servo_direction *= -1

# Check if object is centered
def is_centered(bbox_center_x, bbox_center_y, frame_center_x, frame_center_y, tolerance=30):
    return (abs(bbox_center_x - frame_center_x) <= tolerance) and (abs(bbox_center_y - frame_center_y) <= tolerance)

# Run function with two servo states
def run(model: str, max_results: int, score_threshold: float, 
        camera_id: int, width: int, height: int) -> None:
    
    global telegram_sending, telegram_thread

    frame_center_x = width // 2
    frame_center_y = height // 2
    detection_result_list = []
    detection_count = 0

    last_detection_time = time.time()
    time_between_detections = 0

    centered = False

    def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
        nonlocal detection_count, last_detection_time, time_between_detections
        detection_result_list.append(result)
        if len(result.detections) >= 2:
            detection_count += 1

        current_time = time.time()
        time_between_detections = current_time - last_detection_time
        last_detection_time = current_time
        print(f"Time between detections: {time_between_detections:.2f} seconds")

    base_options = python.BaseOptions(model_asset_path=model)
    options = vision.ObjectDetectorOptions(base_options=base_options,
                                           running_mode=vision.RunningMode.LIVE_STREAM,
                                           max_results=max_results, score_threshold=score_threshold,
                                           result_callback=save_result)
    detector = vision.ObjectDetector.create_from_options(options)

    try:
        while True:
            im = picam2.capture_array()
            image = cv2.resize(im, (width, height))
            image = cv2.flip(image, -1)
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
            detector.detect_async(mp_image, time.time_ns() // 1_000_000)

            if detection_count < 2:
                move_servo_continuous()
            else:
                if detection_result_list:
                    current_frame = visualize(image, detection_result_list[0])
                    for detection in detection_result_list[0].detections:
                        bbox = detection.bounding_box
                        bbox_center_x = bbox.origin_x + bbox.width // 2
                        bbox_center_y = bbox.origin_y + bbox.height // 2

                        move_servo_to_center(bbox_center_x, bbox_center_y, frame_center_x, frame_center_y)

                        if is_centered(bbox_center_x, bbox_center_y, frame_center_x, frame_center_y):
                            if not centered:
                                print("Đối tượng đã ở trung tâm")
                                led.off()
                                relay.on()

                                if not telegram_sending:
                                    telegram_sending = True
                                    telegram_thread = threading.Thread(target=send_telegram_message)
                                    telegram_thread.start()
                                
                                centered = True
                        else:
                            if centered:
                                print("Đối tượng không còn ở trung tâm")
                                relay.off()
                                led.on()

                                if telegram_sending:
                                    telegram_sending = False
                                    if telegram_thread is not None:
                                        telegram_thread.join()
                                    telegram_thread = None
                                
                                centered = False

                    detection_result_list.clear()
                else:
                    led.off()

            show_fps(image)

            cv2.imshow('object_detection', image)
            
            if cv2.waitKey(1) == 27:
                break

    except KeyboardInterrupt:
        print("Đã nhận tín hiệu dừng từ bàn phím")
    finally:
        detector.close()
        relay.off()
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--model', help='Path of the object detection model.', default='best.tflite')
    parser.add_argument('--maxResults', help='Max number of detection results.', default=5, type=int)
    parser.add_argument('--scoreThreshold', help='The score threshold of detection results.', type=float, default=0.6)
    parser.add_argument('--cameraId', help='Id of camera.', type=int, default=0)
    parser.add_argument('--frameWidth', help='Width of frame to capture from camera.', type=int, default=640)
    parser.add_argument('--frameHeight', help='Height of frame to capture from camera.', type=int, default=480)
    args = parser.parse_args()

    run(args.model, args.maxResults, args.scoreThreshold, args.cameraId, args.frameWidth, args.frameHeight)

if __name__ == '__main__':
    main()
