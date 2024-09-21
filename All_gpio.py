import argparse
import sys
import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from utils import visualize
from picamera2 import Picamera2
import RPi.GPIO as GPIO
from time import sleep
import threading
import asyncio
from telegram import Bot

# ========== Cấu hình Telegram ==========
# Thay 'YOUR_TOKEN' bằng token của bot mà bạn đã nhận được từ BotFather
TOKEN = '7377344059:AAGfmth-0UmRiyGicVb0BcyY_uEihWYTQgk'
# Thay 'YOUR_CHAT_ID' bằng chat ID của người nhận
CHAT_ID = '6888591504'

bot = Bot(token=TOKEN)

# Biến để kiểm soát luồng gửi tin nhắn
telegram_sending = False
telegram_thread = None

def send_telegram_message():
    """
    Hàm này chạy trong một luồng riêng để gửi tin nhắn Telegram mỗi 2 giây.
    """
    global telegram_sending
    asyncio.run(send_message_periodically())

async def send_message_periodically():
    while telegram_sending:
        try:
            await bot.send_message(chat_id=CHAT_ID, text="Nhà bạn đang có cháy!!!")
            print("Đã gửi tin nhắn Telegram")
        except Exception as e:
            print(f"Lỗi khi gửi tin nhắn Telegram: {e}")
        await asyncio.sleep(2)  # Đợi 2 giây trước khi gửi lại

# ========== Cấu hình GPIO ==========
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

# Servo setup for X and Y axes
servo_pin_x = 18  # GPIO pin for X-axis servo
servo_pin_y = 14  # GPIO pin for Y-axis servo
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin_x, GPIO.OUT)
GPIO.setup(servo_pin_y, GPIO.OUT)

# LED setup
led_pin = 15  # GPIO pin for LED
GPIO.setup(led_pin, GPIO.OUT)
GPIO.output(led_pin, GPIO.LOW)

# Relay setup
relay_pin = 17  # GPIO pin cho relay 5V (thay đổi nếu cần)
GPIO.setup(relay_pin, GPIO.OUT)
GPIO.output(relay_pin, GPIO.LOW)  # Tắt relay ban đầu

# Initialize PWM for both servos (50Hz)
pwm_x = GPIO.PWM(servo_pin_x, 50)
pwm_y = GPIO.PWM(servo_pin_y, 50)
pwm_x.start(0)
pwm_y.start(0)

# Variables to track servo angle direction for both servos
servo_angle_x = 90  # Start at middle position
servo_angle_y = 90
servo_direction_x = 1  # 1 for increasing, -1 for decreasing
servo_direction_y = 1

# Servo continuous movement variables
servo_angle = 0  # Initial servo angle for continuous movement
servo_direction = 1  # 1 for increasing, -1 for decreasing

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

# Function to set servo angle for both X and Y axes
def set_angle(servo_pin, angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin, True)
    if servo_pin == servo_pin_x:
        pwm_x.ChangeDutyCycle(duty)
    else:
        pwm_y.ChangeDutyCycle(duty)
    sleep(0.08)
    GPIO.output(servo_pin, False)
    if servo_pin == servo_pin_x:
        pwm_x.ChangeDutyCycle(0)
    else:
        pwm_y.ChangeDutyCycle(0)

# Move servos based on detection box position
def move_servo_to_center(bbox_center_x, bbox_center_y, frame_center_x, frame_center_y):
    global servo_angle_x, servo_angle_y
    tolerance = 30  # Adjust this value if needed

    # X-axis control
    if abs(bbox_center_x - frame_center_x) > tolerance:
        if bbox_center_x < frame_center_x:
            new_angle_x = max(servo_angle_x - 5, 0)  # Prevent angle from going below 0
            set_angle(servo_pin_x, new_angle_x)
            print("move to left")
        else:
            new_angle_x = min(servo_angle_x + 5, 180)  # Prevent angle from going above 180
            set_angle(servo_pin_x, new_angle_x)
            print("move to right")
        servo_angle_x = new_angle_x
    
    # Y-axis control
    if abs(bbox_center_y - frame_center_y) > tolerance:
        if bbox_center_y < frame_center_y:
            new_angle_y = min(servo_angle_y + 5, 180)  # Prevent angle from going above 180
            set_angle(servo_pin_y, new_angle_y)
            print("move up")
        else:
            new_angle_y = max(servo_angle_y - 5, 0)  # Prevent angle from going below 0
            set_angle(servo_pin_y, new_angle_y)
            print("move down")
        servo_angle_y = new_angle_y

# Move servo continuously between 0 and 180 degrees
def move_servo_continuous():
    global servo_angle, servo_direction
    set_angle(servo_pin_x, servo_angle)  # Move servo on X axis (pin 18)
    servo_angle += 10 * servo_direction
    if servo_angle >= 180 or servo_angle <= 0:
        servo_direction *= -1

# Hàm kiểm tra xem đối tượng đã ở trung tâm chưa
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

    # Variables to track detection time, initialized within the run function
    last_detection_time = time.time()
    time_between_detections = 0

    # Biến trạng thái để xác định đã ở trung tâm hay chưa
    centered = False

    def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):
        nonlocal detection_count, last_detection_time, time_between_detections
        detection_result_list.append(result)
        if len(result.detections) >= 2:
            detection_count += 1

        # Calculate time between detections
        current_time = time.time()
        time_between_detections = current_time - last_detection_time
        last_detection_time = current_time
        print(f"Time between detections: {time_between_detections:.2f} seconds")

    # Initialize the object detection model
    base_options = python.BaseOptions(model_asset_path=model)
    options = vision.ObjectDetectorOptions(base_options=base_options,
                                           running_mode=vision.RunningMode.LIVE_STREAM,
                                           max_results=max_results, score_threshold=score_threshold,
                                           result_callback=save_result)
    detector = vision.ObjectDetector.create_from_options(options)

    try:
        while True:
            im = picam2.capture_array()
            image = cv2.resize(im, (width, height))  # Use width and height parameters
            image = cv2.flip(image, -1)
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
            detector.detect_async(mp_image, time.time_ns() // 1_000_000)

            # State 1: Servo moves continuously until 2 objects are detected
            if detection_count < 2:
                move_servo_continuous()  # Call continuous movement function
            else:
                # State 2: Adjust servo based on object detection
                if detection_result_list:
                    current_frame = visualize(image, detection_result_list[0])
                    for detection in detection_result_list[0].detections:
                        bbox = detection.bounding_box
                        bbox_center_x = bbox.origin_x + bbox.width // 2
                        bbox_center_y = bbox.origin_y + bbox.height // 2

                        # Move servo to center the detection box
                        move_servo_to_center(bbox_center_x, bbox_center_y, frame_center_x, frame_center_y)

                        # Kiểm tra xem đối tượng đã ở trung tâm chưa
                        if is_centered(bbox_center_x, bbox_center_y, frame_center_x, frame_center_y):
                            if not centered:
                                print("Đối tượng đã ở trung tâm")
                                # Thay đổi chế độ điều khiển LED thành relay
                                GPIO.output(led_pin, GPIO.LOW)  # Tắt LED
                                GPIO.output(relay_pin, GPIO.HIGH)  # Bật relay

                                # Bắt đầu gửi tin nhắn Telegram nếu chưa bắt đầu
                                if not telegram_sending:
                                    telegram_sending = True
                                    telegram_thread = threading.Thread(target=send_telegram_message)
                                    telegram_thread.start()
                                
                                centered = True
                        else:
                            if centered:
                                print("Đối tượng không còn ở trung tâm")
                                # Thay đổi chế độ điều khiển relay thành LED nhấp nháy
                                GPIO.output(relay_pin, GPIO.LOW)  # Tắt relay
                                GPIO.output(led_pin, GPIO.HIGH)  # Bật LED

                                # Dừng gửi tin nhắn Telegram
                                if telegram_sending:
                                    telegram_sending = False
                                    if telegram_thread is not None:
                                        telegram_thread.join()
                                    telegram_thread = None
                                
                                centered = False

                    detection_result_list.clear()
                else:
                    GPIO.output(led_pin, GPIO.LOW)  # Turn off LED

            # Show FPS on the frame
            show_fps(image)

            # Display camera frame
            cv2.imshow('object_detection', image)
            
            if cv2.waitKey(1) == 27:  # Press ESC to exit
                break

    except KeyboardInterrupt:
        print("Đã nhận tín hiệu dừng từ bàn phím")
    finally:
        detector.close()
        pwm_x.stop()
        pwm_y.stop()
        GPIO.output(relay_pin, GPIO.LOW)  # Tắt relay khi kết thúc
        GPIO.cleanup()
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
