import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- Setup I2C and PCA9685 ---
i2c = busio.I2C(SCL, SDA)        # Use Pi’s default SDA (GPIO2) & SCL (GPIO3)
pca = PCA9685(i2c)               # Create PCA9685 object
pca.frequency = 50               # Standard servo frequency

# --- Setup Servo on channel 15 ---
# MG996R usually works well with default pulse widths (500-2500us)
my_servo = servo.Servo(pca.channels[15])

print("Sweeping MG996R servo on channel 15...")

# --- Sweep back and forth ---
while True:
    # Move from 0 to 180 degrees
    for angle in range(0, 181, 5):
        my_servo.angle = angle
        time.sleep(0.05)
    # Move back from 180 to 0 degrees
    for angle in range(180, -1, -5):
        my_servo.angle = angle
        time.sleep(0.05)
