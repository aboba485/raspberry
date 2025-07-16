import pigpio
import time

class MotorController:
    def __init__(self, gpio_pin1=4, gpio_pin2=5):
        """
        Initialize motor controller with two GPIO pins
        gpio_pin1: First control pin (GPIO 4)
        gpio_pin2: Second control pin (GPIO 5)
        """
        self.pi = pigpio.pi()
        self.pin1 = gpio_pin1
        self.pin2 = gpio_pin2
        
        # Set pins as outputs
        self.pi.set_mode(self.pin1, pigpio.OUTPUT)
        self.pi.set_mode(self.pin2, pigpio.OUTPUT)
        
        # Initialize pins to LOW
        self.pi.write(self.pin1, 0)
        self.pi.write(self.pin2, 0)
        
        print(f"Motor controller initialized on GPIO {self.pin1} and {self.pin2}")
    
    def rotate_forward(self, duration=1.0):
        """
        Rotate motor forward for specified duration
        duration: Time in seconds
        """
        print(f"Rotating forward for {duration} seconds...")
        self.pi.write(self.pin1, 1)  # HIGH
        self.pi.write(self.pin2, 0)  # LOW
        time.sleep(duration)
        self.stop()
    
    def rotate_backward(self, duration=1.0):
        """
        Rotate motor backward for specified duration
        duration: Time in seconds
        """
        print(f"Rotating backward for {duration} seconds...")
        self.pi.write(self.pin1, 0)  # LOW
        self.pi.write(self.pin2, 1)  # HIGH
        time.sleep(duration)
        self.stop()
    
    def stop(self):
        """
        Stop motor rotation
        """
        print("Stopping motor...")
        self.pi.write(self.pin1, 0)
        self.pi.write(self.pin2, 0)
    
    def cleanup(self):
        """
        Clean up GPIO resources
        """
        self.stop()
        self.pi.stop()
        print("GPIO cleanup completed")

# Example usage
def main():
    # Create motor controller instance
    motor = MotorController(gpio_pin1=4, gpio_pin2=5)
    
    try:
        # Test motor rotation
        motor.rotate_forward(2.0)    # Forward for 2 seconds
        time.sleep(0.5)              # Brief pause
        motor.rotate_backward(2.0)   # Backward for 2 seconds
        time.sleep(0.5)              # Brief pause
        
        # Multiple short rotations
        for i in range(3):
            motor.rotate_forward(0.5)
            time.sleep(0.2)
            motor.rotate_backward(0.5)
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    
    finally:
        motor.cleanup()

if __name__ == "__main__":
    main()