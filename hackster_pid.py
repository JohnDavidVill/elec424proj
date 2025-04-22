# https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/

speed = 10 # operating speed in % PWM

#Variables to be updated each loop
lastTime = 0 
lastError = 0

# PD constants
Kp = 0.4
Kd = Kp * 0.65

While True:
    now = time.time() # current time variable
    dt = now - lastTime
    deviation = steering_angle - 90 # equivalent to angle_to_mid_deg variable
    error = abs(deviation) 

    if deviation < 5 and deviation > -5: # do not steer if there is a 10-degree error range
        deviation = 0
        error = 0
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        steering.stop()

    elif deviation > 5: # steer right if the deviation is positive
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        steering.start(100)

    elif deviation < -5: # steer left if deviation is negative
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        steering.start(100)

    derivative = kd * (error - lastError) / dt 
    proportional = kp * error
    PD = int(speed + derivative + proportional)

    spd = abs(PD)
    if spd > 25:
       spd = 25

    throttle.start(spd)

    lastError = error
    lastTime = time.time()