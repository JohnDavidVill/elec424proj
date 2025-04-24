# from ChatGPT: https://chatgpt.com/share/680a5525-2f7c-800e-8e72-81692311bbf0
from gpiozero import PWMOutputDevice
import time

# Constants
ESC_GPIO = 18   # GPIO pin connected to ESC
FREQ = 50       # 50 Hz for typical RC ESCs
PERIOD = 1.0 / FREQ  # 20ms PWM period

# Convert pulse width (μs) to PWM value [0.0 - 1.0]
def pulse_to_value(pulse_width_us):
    return pulse_width_us / (PERIOD * 1_000_000)

# Setup PWM device
esc = PWMOutputDevice(ESC_GPIO, frequency=FREQ, initial_value=pulse_to_value(1500))

def set_speed(pulse_us):
    value = pulse_to_value(pulse_us)
    esc.value = value
    print(f"Speed set to: {pulse_us}μs ({value:.3f} duty ratio)")

# ESC Calibration
def calibrate_esc():
    print("Calibrating ESC...")
    set_speed(2000)  # Full throttle
    time.sleep(2)
    set_speed(1000)  # Min throttle
    time.sleep(2)
    print("ESC Calibration complete.")

# Run calibration
calibrate_esc()

try:
    while True:
        user_input = input("Enter speed (1000-2000, 'q' to quit): ")
        if user_input.lower() == 'q':
            break
        try:
            speed = int(user_input)
            if 1000 <= speed <= 2000:
                set_speed(speed)
            else:
                print("Invalid input. Enter between 1000 and 2000.")
        except ValueError:
            print("Please enter a valid number.")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    print("Stopping motor and cleaning up...")
    set_speed(1000)  # Neutral/off
    time.sleep(2)
    esc.close()
