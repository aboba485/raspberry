import RPi.GPIO as GPIO
import time
import smbus

class ServoDriverMotorControl:
    def __init__(self, i2c_address=0x40, channel=0):
        """
        Control motor connected to servo driver channel 0
        
        Connections:
        Raspberry Pi → Servo Driver (PCA9685):
        • 5V → VCC (logic power)
        • GND → GND
        • GPIO 2 (SDA) → SDA
        • GPIO 3 (SCL) → SCL
        • External Power → V+ (motor power)
        
        Motor → Servo Driver:
        • Motor connected to Channel 0 output
        """
        self.i2c_address = i2c_address
        self.channel = channel
        self.bus = None
        
        try:
            # Initialize I2C bus
            self.bus = smbus.SMBus(1)  # I2C bus 1 on Raspberry Pi
            
            # Initialize PCA9685
            self.init_pca9685()
            
            print(f"🤖 Servo Driver initialized at address 0x{i2c_address:02x}")
            print(f"📌 Motor on channel: {channel}")
            
        except Exception as e:
            print(f"❌ Failed to initialize servo driver: {e}")
            print("💡 Make sure I2C is enabled and driver is connected")
    
    def init_pca9685(self):
        """
        Initialize PCA9685 servo driver
        """
        # Reset the device
        self.bus.write_byte_data(self.i2c_address, 0x00, 0x00)
        
        # Set PWM frequency to ~50Hz for servos
        prescale = int(25000000.0 / (4096 * 50.0) - 1)
        
        # Go to sleep mode to set prescaler
        old_mode = self.bus.read_byte_data(self.i2c_address, 0x00)
        sleep_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.i2c_address, 0x00, sleep_mode)
        
        # Set prescaler
        self.bus.write_byte_data(self.i2c_address, 0xFE, prescale)
        
        # Wake up
        self.bus.write_byte_data(self.i2c_address, 0x00, old_mode)
        time.sleep(0.005)
        
        # Enable auto-increment
        self.bus.write_byte_data(self.i2c_address, 0x00, old_mode | 0xA1)
        
        print("✅ PCA9685 initialized")
    
    def set_pwm(self, channel, on_time, off_time):
        """
        Set PWM for specific channel
        channel: 0-15
        on_time: 0-4095 (when to turn on)
        off_time: 0-4095 (when to turn off)
        """
        try:
            # Calculate register addresses for the channel
            base_reg = 0x06 + 4 * channel
            
            # Write ON time (low and high bytes)
            self.bus.write_byte_data(self.i2c_address, base_reg, on_time & 0xFF)
            self.bus.write_byte_data(self.i2c_address, base_reg + 1, on_time >> 8)
            
            # Write OFF time (low and high bytes)  
            self.bus.write_byte_data(self.i2c_address, base_reg + 2, off_time & 0xFF)
            self.bus.write_byte_data(self.i2c_address, base_reg + 3, off_time >> 8)
            
        except Exception as e:
            print(f"❌ Failed to set PWM: {e}")
    
    def set_servo_angle(self, angle):
        """
        Set servo angle (0-180 degrees) for channel 0
        """
        # Convert angle to PWM values
        # Typical servo: 1ms = 0°, 1.5ms = 90°, 2ms = 180°
        # For 50Hz: 1ms = ~204, 1.5ms = ~307, 2ms = ~409
        
        angle = max(0, min(180, angle))
        
        # Calculate PWM value
        min_pulse = 204   # ~1ms
        max_pulse = 409   # ~2ms
        pulse_width = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
        
        # Set PWM (on at 0, off at calculated position)
        self.set_pwm(self.channel, 0, int(pulse_width))
        
        print(f"🎯 Channel {self.channel} set to {angle}° (PWM: {int(pulse_width)})")
    
    def continuous_rotation_speed(self, speed):
        """
        For continuous rotation servos
        speed: -100 to +100 (negative = reverse, positive = forward, 0 = stop)
        """
        speed = max(-100, min(100, speed))
        
        # Convert speed to pulse width
        # Continuous servos: ~1.5ms = stop, <1.5ms = reverse, >1.5ms = forward
        center_pulse = 307  # 1.5ms (stop)
        max_range = 102     # Range for full speed
        
        pulse_width = center_pulse + (speed / 100.0) * max_range
        
        self.set_pwm(self.channel, 0, int(pulse_width))
        
        direction = "STOP" if speed == 0 else ("FORWARD" if speed > 0 else "REVERSE")
        print(f"🔄 Channel {self.channel}: {direction} at {abs(speed)}% speed")
    
    def motor_stop(self):
        """
        Stop motor on channel 0
        """
        self.set_pwm(self.channel, 0, 0)  # Turn off PWM
        print(f"🛑 Channel {self.channel} stopped")
    
    def test_servo_positions(self):
        """
        Test standard servo positions
        """
        print(f"\n🔧 Testing servo positions on channel {self.channel}...")
        
        positions = [0, 45, 90, 135, 180, 90]
        
        for pos in positions:
            print(f"📍 Moving to {pos}°")
            self.set_servo_angle(pos)
            time.sleep(2)
        
        print("✅ Position test complete")
    
    def test_continuous_rotation(self):
        """
        Test continuous rotation (if motor supports it)
        """
        print(f"\n🔄 Testing continuous rotation on channel {self.channel}...")
        
        # Forward
        print("➡️  Forward 50%")
        self.continuous_rotation_speed(50)
        time.sleep(3)
        
        # Stop
        print("⏸️  Stop")
        self.continuous_rotation_speed(0)
        time.sleep(1)
        
        # Reverse
        print("⬅️  Reverse 50%")
        self.continuous_rotation_speed(-50)
        time.sleep(3)
        
        # Stop
        print("🛑 Final stop")
        self.motor_stop()
        
        print("✅ Continuous rotation test complete")
    
    def scan_i2c_devices(self):
        """
        Scan for I2C devices to find servo driver
        """
        print("\n🔍 Scanning I2C bus for devices...")
        
        devices = []
        for addr in range(0x03, 0x78):
            try:
                self.bus.read_byte(addr)
                devices.append(addr)
                print(f"   Found device at 0x{addr:02x}")
            except:
                pass
        
        if not devices:
            print("❌ No I2C devices found!")
            print("💡 Check connections and enable I2C")
        else:
            print(f"✅ Found {len(devices)} device(s)")
            
        return devices

# Alternative: GPIO-based servo driver control
class GPIOServoDriverControl:
    def __init__(self, control_pin1=4, control_pin2=12, servo_channel=0):
        """
        If your "servo driver" uses GPIO control instead of I2C
        """
        self.control_pin1 = control_pin1
        self.control_pin2 = control_pin2
        self.servo_channel = servo_channel
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.control_pin1, GPIO.OUT)
        GPIO.setup(self.control_pin2, GPIO.OUT)
        
        # Create PWM on one of the pins for servo control
        self.pwm = GPIO.PWM(self.control_pin1, 50)  # 50Hz for servo
        self.pwm.start(0)
        
        print(f"🤖 GPIO Servo Driver - Channel {servo_channel}")
        print(f"📌 Control pins: GPIO {control_pin1}, GPIO {control_pin2}")
    
    def set_servo_angle(self, angle):
        """
        Set servo angle using GPIO PWM
        """
        angle = max(0, min(180, angle))
        
        # Convert angle to duty cycle (servo standard)
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        
        self.pwm.ChangeDutyCycle(duty_cycle)
        print(f"🎯 Channel {self.servo_channel} angle: {angle}°")
        
        time.sleep(0.5)
    
    def cleanup(self):
        """
        Clean up GPIO
        """
        self.pwm.stop()
        GPIO.cleanup()
        print("✅ GPIO cleaned up")

def main():
    """
    Main program to test servo driver motor control
    """
    print("🤖 SERVO DRIVER MOTOR CONTROL")
    print("="*50)
    print("📌 Motor connected to servo driver channel 0")
    
    choice = input("""
Select driver type:
1 - I2C Servo Driver (PCA9685-style)
2 - GPIO Servo Driver  
3 - Scan I2C devices first
4 - Test both types

Enter (1-4): """)
    
    try:
        if choice == "1":
            # I2C servo driver
            driver = ServoDriverMotorControl(channel=0)
            
            test_choice = input("""
Test type:
a - Servo positions (0-180°)
b - Continuous rotation
c - Both tests

Enter (a-c): """).lower()
            
            if test_choice == "a":
                driver.test_servo_positions()
            elif test_choice == "b":
                driver.test_continuous_rotation()
            else:
                driver.test_servo_positions()
                time.sleep(2)
                driver.test_continuous_rotation()
        
        elif choice == "2":
            # GPIO servo driver
            driver = GPIOServoDriverControl(control_pin1=4, control_pin2=12, servo_channel=0)
            
            try:
                print("Testing GPIO servo control...")
                positions = [0, 90, 180, 90]
                
                for pos in positions:
                    driver.set_servo_angle(pos)
                    time.sleep(2)
                    
            finally:
                driver.cleanup()
        
        elif choice == "3":
            # Scan I2C first
            scanner = ServoDriverMotorControl()
            devices = scanner.scan_i2c_devices()
            
            if 0x40 in devices:
                print("✅ Found PCA9685 at default address 0x40")
            elif devices:
                print(f"💡 Try address 0x{devices[0]:02x}")
        
        elif choice == "4":
            print("Testing both types...")
            
            # Try I2C first
            try:
                print("\n1️⃣  Trying I2C servo driver...")
                driver = ServoDriverMotorControl(channel=0)
                driver.test_servo_positions()
            except:
                print("❌ I2C failed, trying GPIO...")
                
                # Try GPIO
                driver = GPIOServoDriverControl(servo_channel=0)
                try:
                    driver.test_servo_positions = lambda: [driver.set_servo_angle(a) for a in [0, 90, 180]]
                    driver.test_servo_positions()
                finally:
                    driver.cleanup()
    
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted")
    except Exception as e:
        print(f"❌ Error: {e}")
    
    print("\n💡 If motor doesn't move:")
    print("1. Check if I2C is enabled: sudo raspi-config")
    print("2. Verify servo driver type (PCA9685 vs other)")
    print("3. Ensure external power connected to driver")
    print("4. Check motor is on correct channel (0)")

if __name__ == "__main__":
    main()