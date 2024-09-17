import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

# Tạo tín hiệu PWM với tần số 50Hz
pwm = GPIO.PWM(17, 50)
pwm.start(0)

try:
    while True:
        # Ví dụ, chuyển servo đến góc 90 độ
        pwm.ChangeDutyCycle(7.5) # 7.5% duty cycle tương đương với góc 90 độ
        time.sleep(1)
        
        # Chuyển servo đến góc 0 độ
        pwm.ChangeDutyCycle(2.5) # 2.5% duty cycle tương đương với góc 0 độ
        time.sleep(1)
        
        # Chuyển servo đến góc 180 độ
        pwm.ChangeDutyCycle(12.5) # 12.5% duty cycle tương đương với góc 180 độ
        time.sleep(1)
except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
