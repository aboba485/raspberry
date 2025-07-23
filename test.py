import RPi.GPIO as GPIO
import time

class PowerbankMotorTest:
    def __init__(self, control_pin1=4, control_pin2=12):
        """
        Test motor with powerbank as external supply
        """
        self.control_pin1 = control_pin1
        self.control_pin2 = control_pin2
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.control_pin1, GPIO.OUT)
        GPIO.setup(self.control_pin2, GPIO.OUT)
        
        # Start with motor off
        GPIO.output(self.control_pin1, GPIO.LOW)
        GPIO.output(self.control_pin2, GPIO.LOW)
        
        print("🔋 Powerbank Motor Test Setup")
        print(f"📌 Control pins: GPIO {self.control_pin1}, GPIO {self.control_pin2}")
    
    def test_powerbank_connection(self):
        """
        Test if powerbank provides enough power
        """
        print("\n🔍 POWERBANK CONNECTION TEST")
        print("="*50)
        
        print("📋 Before starting, verify your connections:")
        print("   Powerbank USB → Cut cable → Driver VIN")
        print("   USB Red wire (+5V) → Driver VIN/VM")
        print("   USB Black wire (GND) → Driver GND + Pi GND")
        print("   Pi 5V → Driver VCC (logic power)")
        print("   GPIO 4 → Driver IN1")
        print("   GPIO 12 → Driver IN2")
        
        input("\nPress Enter when connections are ready...")
        
        print("\n1️⃣  Testing motor direction 1...")
        print("👀 WATCH THE MOTOR - does it move?")
        GPIO.output(self.control_pin1, GPIO.HIGH)
        GPIO.output(self.control_pin2, GPIO.LOW)
        
        time.sleep(3)
        
        print("\n2️⃣  Testing motor direction 2...")
        print("👀 WATCH THE MOTOR - does it change direction?")
        GPIO.output(self.control_pin1, GPIO.LOW)
        GPIO.output(self.control_pin2, GPIO.HIGH)
        
        time.sleep(3)
        
        print("\n3️⃣  Stopping motor...")
        GPIO.output(self.control_pin1, GPIO.LOW)
        GPIO.output(self.control_pin2, GPIO.LOW)
        
        # Get user feedback
        result = input("\nDid the motor move? (y/n): ").lower()
        
        if result == 'y':
            print("✅ SUCCESS! Powerbank works with your setup!")
            return True
        else:
            print("❌ Motor didn't move. Let's troubleshoot...")
            return False
    
    def troubleshoot_powerbank(self):
        """
        Troubleshooting steps for powerbank setup
        """
        print("\n🔧 POWERBANK TROUBLESHOOTING")
        print("="*40)
        
        print("Common issues with powerbank setup:")
        
        print("\n1️⃣  Voltage too low (5V vs 6-12V needed):")
        print("   • Some drivers need minimum 6V")
        print("   • Motor may be too weak to start")
        print("   • Try a different driver or boost converter")
        
        print("\n2️⃣  Current limiting:")
        print("   • Powerbanks limit current to ~2-3A")
        print("   • Motor startup current might be higher")
        print("   • Powerbank may shut off if overloaded")
        
        print("\n3️⃣  Connection issues:")
        print("   • USB cable must be properly cut/stripped")
        print("   • Red wire = +5V, Black wire = GND")
        print("   • All grounds must be connected together")
        
        print("\n4️⃣  Driver compatibility:")
        print("   • Some drivers need 6-12V minimum")
        print("   • L298N works with 5V but reduced power")
        print("   • Check your driver's minimum voltage")
    
    def test_with_load_simulation(self):
        """
        Test with gradual load increase
        """
        print("\n⚡ GRADUAL POWER TEST")
        print("="*30)
        
        print("Testing with short pulses first...")
        
        for i in range(5):
            print(f"Pulse {i+1}/5 - Duration: {0.2 * (i+1):.1f}s")
            
            GPIO.output(self.control_pin1, GPIO.HIGH)
            GPIO.output(self.control_pin2, GPIO.LOW)
            time.sleep(0.2 * (i+1))
            
            GPIO.output(self.control_pin1, GPIO.LOW)
            GPIO.output(self.control_pin2, GPIO.LOW)
            time.sleep(0.5)
            
            moved = input(f"Did motor move on pulse {i+1}? (y/n): ").lower()
            if moved == 'y':
                print(f"✅ Motor responds at {0.2 * (i+1):.1f}s duration")
                return True
        
        print("❌ Motor not responding to any pulses")
        return False
    
    def cleanup(self):
        """
        Clean up GPIO
        """
        GPIO.output(self.control_pin1, GPIO.LOW)
        GPIO.output(self.control_pin2, GPIO.LOW)
        GPIO.cleanup()
        print("✅ GPIO cleaned up")

def powerbank_solutions():
    """
    Alternative solutions if powerbank doesn't work
    """
    print("\n💡 SOLUTIONS IF POWERBANK DOESN'T WORK")
    print("="*50)
    
    print("1️⃣  Use USB Boost Converter:")
    print("   • Converts 5V → 9V or 12V")
    print("   • Available on Amazon/eBay")
    print("   • Small module, easy to use")
    
    print("\n2️⃣  Use 9V Battery:")
    print("   • Simple 9V battery + battery holder")
    print("   • Higher voltage = better motor performance")
    print("   • Connect 9V(+) to driver VIN, 9V(-) to GND")
    
    print("\n3️⃣  Use Power Adapter:")
    print("   • 9V-12V wall adapter")
    print("   • More reliable than batteries")
    print("   • Check current rating (2A+ recommended)")
    
    print("\n4️⃣  Different Driver:")
    print("   • Some drivers work better with 5V")
    print("   • Motor driver HATs designed for Pi")
    print("   • Check if your driver supports 5V operation")

def main():
    """
    Main powerbank testing program
    """
    print("🔋 POWERBANK MOTOR TESTING")
    print("="*40)
    
    tester = PowerbankMotorTest()
    
    try:
        choice = input("""
Test options:
1 - Full powerbank connection test
2 - Gradual power test (pulses)
3 - View troubleshooting info
4 - View alternative solutions

Enter (1-4): """)
        
        if choice == "1":
            success = tester.test_powerbank_connection()
            if not success:
                tester.troubleshoot_powerbank()
        
        elif choice == "2":
            success = tester.test_with_load_simulation()
            if not success:
                powerbank_solutions()
        
        elif choice == "3":
            tester.troubleshoot_powerbank()
        
        elif choice == "4":
            powerbank_solutions()
        
        else:
            print("Running full test...")
            success = tester.test_powerbank_connection()
            if not success:
                tester.troubleshoot_powerbank()
                powerbank_solutions()
    
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted")
    
    finally:
        tester.cleanup()

if __name__ == "__main__":
    main()
    
    print("\n🎯 QUICK CHECKLIST:")
    print("✓ Powerbank connected to driver VIN")
    print("✓ All grounds connected together") 
    print("✓ GPIO signals reaching driver")
    print("✓ Motor connected to driver output")
    print("✓ Driver minimum voltage requirements met")