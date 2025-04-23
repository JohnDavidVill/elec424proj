import RPi.GPIO as GPIO
import time

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(19, GPIO.OUT)

p = GPIO.PWM(19, 50)

p.start(1.5)
time.sleep(5)
p.ChangeDutyCycle(1.2)
time.sleep(5)
p.ChangeDutyCycle(1.8)
time.sleep(5)
p.ChangeDutyCycle(1.2)
time.sleep(5)

p.stop()
GPIO.cleanup()
