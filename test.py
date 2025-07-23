import RPi.GPIO as GPIO
import time
import threading

class ServoDriverMotor:
    def __init__(self, control_pin1=4, control_pin2=12, servo_channel=0):
        """
        Motor control through servo driver
        
        Connections:
        Raspberry Pi → Servo Driver:
        • 5V → VCC
        • GND → GND  
        • GPIO 4 → Control Pin 1
        • GPIO 12 → Control Pin 2
        
        Motor → Servo Driver:
        • Motor connected to channel 0 on driver
        """
        self.control_pin1 = control_pin1  # GPIO 4
        self.control_pin2 = control_pin2  # GPIO 12
        self.servo_channel = servo_channel
        self.is_running = False
        self.motor_thread = None
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.control_pin1, GPIO.OUT)
        GPIO.setup(self.control_pin2, GPIO.OUT)
        
        # Initialize pins to LOW
        GPIO.output(self.control_pin1, GPIO.LOW)
        GPIO.output(self.control_pin2, GPIO.LOW)
        
        print(f"🤖 Servo Driver Motor Controller initialized")
        print(f"📌 Control pins: GPIO {self.control_pin1}, GPIO {self.control_pin2}")
        print(f"📌 Motor on driver channel: {self.servo_channel}")
    
    def motor_forward(self, duration=None):
        """
        Run motor forward
        duration: time in seconds (None = indefinite)
        """
        print("➡️  Motor Forward")
        GPIO.output(self.control_pin1, GPIO.HIGH)
        GPIO.output(self.control_pin2, GPIO.LOW)
        
        if duration:
            time.sleep(duration)
            self.motor_stop()
    
    def motor_backward(self, duration=None):
        """
        Run motor backward  
        duration: time in seconds (None = indefinite)
        """
        print("⬅️  Motor Backward")
        GPIO.output(self.control_pin1, GPIO.LOW)
        GPIO.output(self.control_pin2, GPIO.HIGH)
        
        if duration:
            time.sleep(duration)
            self.motor_stop()
    
    def motor_stop(self):
        """
        Stop motor
        """
        print("🛑 Motor Stopped")
        GPIO.output(self.control_pin1, GPIO.LOW)
        GPIO.output(self.control_pin2, GPIO.LOW)
        self.is_running = False
    
    def motor_continuous(self, direction="forward"):
        """
        Run motor continuously in background thread
        direction: "forward" or "backward"
        """
        if self.is_running:
            print("⚠️  Motor already running!")
            return
        
        self.is_running = True
        
        def continuous_run():
            print(f"🔄 Continuous motor run: {direction}")
            while self.is_running:
                if direction == "forward":
                    GPIO.output(self.control_pin1, GPIO.HIGH)
                    GPIO.output(self.control_pin2, GPIO.LOW)
                else:
                    GPIO.output(self.control_pin1, GPIO.LOW)
                    GPIO.output(self.control_pin2, GPIO.HIGH)
                time.sleep(0.1)
        
        self.motor_thread = threading.Thread(target=continuous_run)
        self.motor_thread.daemon = True
        self.motor_thread.start()
    
    def reverse_direction(self):
        """
        Reverse motor direction while running
        """
        if self.is_running:
            print("🔄 Reversing direction")
            # Read current state
            pin1_state = GPIO.input(self.control_pin1)
            pin2_state = GPIO.input(self.control_pin2)
            
            # Swap states
            GPIO.output(self.control_pin1, pin2_state)
            GPIO.output(self.control_pin2, pin1_state)
        else:
            print("⚠️  Motor not running")
    
    def test_motor(self):
        """
        Basic motor test sequence
        """
        print("🔧 Testing motor connection...")
        
        # Test forward
        print("1️⃣  Forward test (3 seconds)")
        self.motor_forward(3)
        time.sleep(0.5)
        
        # Test backward
        print("2️⃣  Backward test (3 seconds)")
        self.motor_backward(3)
        time.sleep(0.5)
        
        # Test stop
        print("3️⃣  Stop test")
        self.motor_stop()
        
        print("✅ Motor test completed")
    
    def cleanup(self):
        """
        Clean up GPIO resources
        """
        self.motor_stop()
        time.sleep(0.5)
        GPIO.cleanup()
        print("✅ GPIO cleaned up")

def keyboard_control():
    """
    Keyboard control for motor
    """
    motor = ServoDriverMotor(control_pin1=4, control_pin2=12, servo_channel=0)
    
    print("\n" + "="*50)
    print("🎮 MOTOR CONTROL VIA SERVO DRIVER")
    print("="*50)
    print("Commands:")
    print("f + Enter - FORWARD")
    print("b + Enter - BACKWARD")
    print("s + Enter - STOP")
    print("c + Enter - CONTINUOUS FORWARD")
    print("r + Enter - REVERSE DIRECTION")
    print("t + Enter - TEST MOTOR")
    print("x + Enter - EXIT")
    print("="*50)
    
    try:
        while True:
            command = input("Command: ").lower().strip()
            
            if command == 'f':
                motor.motor_forward(2)  # Run for 2 seconds
            elif command == 'b':
                motor.motor_backward(2)  # Run for 2 seconds
            elif command == 's':
                motor.motor_stop()
            elif command == 'c':
                motor.motor_continuous("forward")
            elif command == 'r':
                motor.reverse_direction()
            elif command == 't':
                motor.test_motor()
            elif command == 'x':
                print("👋 Exiting...")
                break
            else:
                print("❓ Use: f, b, s, c, r, t, x")
                
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    finally:
        motor.cleanup()

def demo_sequence():
    """
    Demonstration sequence
    """
    motor = ServoDriverMotor()
    
    try:
        print("\n=== MOTOR DEMO SEQUENCE ===")
        
        # Basic test
        motor.test_motor()
        
        print("\n🔄 Continuous operation demo")
        
        # Continuous forward
        print("▶️  Continuous forward (5 seconds)")
        motor.motor_continuous("forward")
        time.sleep(5)
        
        # Reverse direction
        print("🔄 Reversing direction")
        motor.reverse_direction()
        time.sleep(3)
        
        # Stop
        motor.motor_stop()
        
        print("✅ Demo completed!")
        
    finally:
        motor.cleanup()

def quick_test():
    """
    Quick motor test
    """
    motor = ServoDriverMotor(control_pin1=4, control_pin2=12)
    
    try:
        print("🚀 Quick Test")
        motor.motor_forward(1)
        time.sleep(0.5)
        motor.motor_backward(1)
        motor.motor_stop()
        print("✅ Quick test done")
    finally:
        motor.cleanup()

# Main program
if __name__ == "__main__":
    print("🤖 SERVO DRIVER MOTOR CONTROLLER")
    print("="*50)
    print("📌 Connections:")
    print("   Raspberry Pi → Servo Driver:")
    print("   • 5V → VCC")
    print("   • GND → GND")
    print("   • GPIO 4 → Control Pin 1")
    print("   • GPIO 12 → Control Pin 2")
    print("   • Motor → Driver Channel 0")
    print("="*50)
    
    choice = input("""
Select mode:
1 - Keyboard Control
2 - Demo Sequence  
3 - Quick Test
4 - Custom Test

Enter number (1-4): """)
    
    try:
        if choice == "1":
            keyboard_control()
        elif choice == "2":
            demo_sequence()
        elif choice == "3":
            quick_test()
        elif choice == "4":
            # Custom test area
            motor = ServoDriverMotor()
            try:
                print("🔧 Custom test - modify this section as needed")
                motor.test_motor()
                
                # Add your custom code here
                print("Adding 5-second continuous run...")
                motor.motor_continuous("forward")
                time.sleep(5)
                motor.motor_stop()
                
            finally:
                motor.cleanup()
        else:
            print("❌ Invalid choice, running quick test")
            quick_test()
            
    except Exception as e:
        print(f"❌ Error: {e}")
        print("💡 Check your connections and try again")
    finally:
        try:
            GPIO.cleanup()
        except:
            pass

# Quick functions for direct use
def run_forward(seconds=2):
    """Quick forward run"""
    motor = ServoDriverMotor()
    try:
        motor.motor_forward(seconds)
    finally:
        motor.cleanup()

def run_backward(seconds=2):
    """Quick backward run"""
    motor = ServoDriverMotor()
    try:
        motor.motor_backward(seconds)
    finally:
        motor.cleanup()

# Usage examples:
# run_forward(3)    # Run forward for 3 seconds
# run_backward(2)   # Run backward for 2 seconds
# quick_test()      # Quick test sequence