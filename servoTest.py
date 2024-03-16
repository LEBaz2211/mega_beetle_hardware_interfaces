import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

# Define control pins for the first stepper motor
control_pins1 = [29, 31, 33, 35]

# Define control pins for the second stepper motor
control_pins2 = [36, 38, 40, 32]

# Set up pins for the first stepper motor
for pin in control_pins1:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

# Set up pins for the second stepper motor
for pin in control_pins2:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)
# Define sequences for both forward and reverse direction
halfstep_seq = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

# Reverse sequence
halfstep_seq_reverse = halfstep_seq[::-1]

# Number of steps for each stepper motor
steps = 10

# Loop for stepping both motors
for i in range(steps):
    for halfstep in range(8):
        # Step the first motor forward
        for pin in range(4):
            GPIO.output(control_pins1[pin], halfstep_seq[halfstep][pin])
        # Step the second motor forward
        for pin in range(4):
            GPIO.output(control_pins2[pin], halfstep_seq[halfstep][pin])
        time.sleep(0.001)

# Reverse direction
time.sleep(0.5)  # Delay before reversing direction

# Loop for stepping both motors in reverse
for i in range(steps):
    for halfstep in range(8):
        # Step the first motor in reverse
        for pin in range(4):
            GPIO.output(control_pins1[pin], halfstep_seq_reverse[halfstep][pin])
        # Step the second motor in reverse
        for pin in range(4):
            GPIO.output(control_pins2[pin], halfstep_seq_reverse[halfstep][pin])
        time.sleep(0.001)

# Clean up GPIO
GPIO.cleanup()

