import RPi.GPIO as GPIO
import time

# Этот код сразу работает - ничего устанавливать не нужно!
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)
GPIO.setup(5, GPIO.OUT)

print("🔄 Testing motor...")

# Forward
GPIO.output(4, GPIO.HIGH)
GPIO.output(5, GPIO.LOW)
time.sleep(2)

# Stop
GPIO.output(4, GPIO.LOW)
GPIO.output(5, GPIO.LOW)
time.sleep(0.5)

# Backward  
GPIO.output(4, GPIO.LOW)
GPIO.output(5, GPIO.HIGH)
time.sleep(2)

# Stop
GPIO.output(4, GPIO.LOW)
GPIO.output(5, GPIO.LOW)

GPIO.cleanup()
print("✅ Test done!")