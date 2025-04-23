import RPi.GPIO as GPIO
import time
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.OUT)

def blink():
    p = GPIO.PWM(18, 1)
    p.start(1)
    input('Press return to stop:')   # use raw_input for Python 2
    p.stop()
    GPIO.cleanup()

def intensity():

    p = GPIO.PWM(18, 50)  # channel=12 frequency=50Hz
    p.start(0)
    try:
        while 1:
            for dc in range(0, 101, 5):
                p.ChangeDutyCycle(dc)
                time.sleep(0.1)
    #         for dc in range(100, -1, -5):
    #             p.ChangeDutyCycle(dc)
    #             time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    
    p.stop()
    GPIO.cleanup()

blink()
intensity()