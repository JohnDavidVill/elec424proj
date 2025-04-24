import RPi.GPIO as GPIO
import time

#GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
print("Initialized")
p = GPIO.PWM(19, 50)
p2 = GPIO.PWM(18, 50)

print("PWM Setup")

p.start(7.5)
p2.start(7.5)
print("PWM Started")

time.sleep(5)

p.ChangeDutyCycle(6)
p2.ChangeDutyCycle(6)

print("PWM Changed")

time.sleep(5)

p.ChangeDutyCycle(9)
p2.ChangeDutyCycle(9)
print("PWM Changed")

time.sleep(5)

p.ChangeDutyCycle(6)
p2.ChangeDutyCycle(6)

print("PWM Changed")

time.sleep(5)

p.stop()
p2.stop()
GPIO.cleanup()
print("Program Done")




