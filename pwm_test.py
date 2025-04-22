import RPi.GPIO as GPIO
import time

# Use BCM numbering (pin 18 = BCM 18, pin 19 = BCM 19)
GPIO.setmode(GPIO.BCM)  
GPIO.setwarnings(False)

# Set up pins (BCM 18 for speed, BCM 19 for steering)
GPIO.setup(18, GPIO.OUT)  # Pin 12 (BOARD) = BCM 18
GPIO.setup(19, GPIO.OUT)  # Pin 13 (BOARD) = BCM 19

# Initialize PWM at 50Hz (for motors/servos)
speed_pwm = GPIO.PWM(18, 50)    # Speed motor (BCM 18)
steering_pwm = GPIO.PWM(19, 50)  # Steering servo (BCM 19)

def test_pwm_with_led(pin):
    """Test PWM output on a pin using an LED"""
    p = GPIO.PWM(pin, 50)  # 50Hz (as per motor specs)
    p.start(0)  # Start with 0% duty cycle
    try:
        print(f"Testing PWM on BCM {pin} (LED should brighten/dim):")
        while True:
            # Sweep duty cycle 5% → 10% (motor range)
            for dc in [5, 7.5, 10, 7.5, 5]:  
                p.ChangeDutyCycle(dc)
                print(f"Duty cycle: {dc}%")
                time.sleep(2)
    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()

# Test BCM 18 (speed motor pin) with LED
test_pwm_with_led(18)  
