# Citation: User raja_961, “Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV”. Instructables. URL: https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/ 
# Import the RPi.GPIO module to control the GPIO pins of the Raspberry Pi
import RPi.GPIO as GPIO

# Import the time module to add delays
import time

# Reset all GPIO settings to default
GPIO.cleanup()

# Use Broadcom (BCM) pin numbering scheme
GPIO.setmode(GPIO.BCM)

# Disable GPIO warnings
GPIO.setwarnings(False)

# Set GPIO pin 19 as an output pin, specifically for Servo control
GPIO.setup(19, GPIO.OUT)

# Set GPIO pin 18 as an output pin, specifically for Motor control
GPIO.setup(18, GPIO.OUT)

# Print a message to confirm initialization
print("Initialized")

# Create a PWM object on pin 18 with a frequency of 50Hz
p = GPIO.PWM(18, 50)

# Create a PWM object on pin 19 with a frequency of 50Hz
p2 = GPIO.PWM(19, 50)

# Print that the RC car is moving forward
print("move forward")

# Start PWM on pin 18 with a duty cycle of 8% (Note, we are using the big car instead of the small car so the duty cycle is different. anything below 8% causes the wheels to either jitter or not move at all)
p.start(8)

# Start PWM on pin 19 with a duty cycle of 7.5% (Servo control is identical on big car and small car, 7.5 makes wheels point straight)
p2.start(7.5)

# Wait for 2.5 seconds
time.sleep(2.5)

# Stop PWM signal on pin 18 by setting duty cycle to 0% (makes wheels stop spinning)
p.ChangeDutyCycle(0)

# Print a message indicating a stop
print("stop")

# Wait for 2.5 seconds
time.sleep(2.5)

# Stop PWM signal on pin 18
p.ChangeDutyCycle(0)

# Set PWM on pin 19 to 9% to turn left
p2.ChangeDutyCycle(9)

# Print that the RC car is turning left
print("left")

# Wait for 2.5 seconds
time.sleep(2.5)

# Set PWM on pin 19 to 6% to turn right
p2.ChangeDutyCycle(6)

# Print that the RC car is turning right
print("right")

# Wait for 2.5 seconds
time.sleep(2.5)

# Reset PWM on pin 19 to neutral 
p2.ChangeDutyCycle(7.5)

# Print that the RC car is going straight
print("straight")

# Wait again before stopping
time.sleep(2.5)

# Stop PWM on pin 18 completely
p.stop()

# Stop PWM on pin 19 completely
p2.stop()

# Clean up all used GPIO pins
GPIO.cleanup()

# Final confirmation message
print("Program Done")
