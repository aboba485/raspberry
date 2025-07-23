import RPi.GPIO as GPIO
import time
import sys

# Вариант 1: I2C серводрайвер (PCA9685)
try:
    import smbus
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    print("⚠️  smbus не установлен. Используйте: sudo apt install python3-smbus")

class I2CServoDriver:
    def __init__(self, i2c_address=0x40, channel=0):
        """
        Серводрайвер через I2C (например PCA9685)
        
        Подключение:
        Raspberry Pi 5 → Серводрайвер:
        • 5V → VCC
        • GND → GND  
        • GPIO 2 (SDA) → SDA
        • GPIO 3 (SCL) → SCL
        • Паурбанк → V+ (питание моторов)
        
        Мотор → Канал 0 драйвера
        """
        if not I2C_AVAILABLE:
            raise Exception("I2C библиотека недоступна")
            
        self.i2c_address = i2c_address
        self.channel = channel
        self.bus = smbus.SMBus(1)
        
        print(f"🤖 I2C серводрайвер: адрес 0x{i2c_address:02x}, канал {channel}")
        
        # Инициализация PCA9685
        self.init_driver()
    
    def init_driver(self):
        """Инициализация драйвера"""
        try:
            # Сброс
            self.bus.write_byte_data(self.i2c_address, 0x00, 0x00)
            
            # Установка частоты PWM ~50Hz
            prescale = int(25000000.0 / (4096 * 50.0) - 1)
            
            old_mode = self.bus.read_byte_data(self.i2c_address, 0x00)
            sleep_mode = (old_mode & 0x7F) | 0x10
            self.bus.write_byte_data(self.i2c_address, 0x00, sleep_mode)
            self.bus.write_byte_data(self.i2c_address, 0xFE, prescale)
            self.bus.write_byte_data(self.i2c_address, 0x00, old_mode)
            time.sleep(0.005)
            self.bus.write_byte_data(self.i2c_address, 0x00, old_mode | 0xA1)
            
            print("✅ I2C драйвер инициализирован")
        except Exception as e:
            print(f"❌ Ошибка инициализации I2C: {e}")
    
    def set_pwm(self, on_time, off_time):
        """Установка PWM для канала 0"""
        try:
            base_reg = 0x06 + 4 * self.channel
            self.bus.write_byte_data(self.i2c_address, base_reg, on_time & 0xFF)
            self.bus.write_byte_data(self.i2c_address, base_reg + 1, on_time >> 8)
            self.bus.write_byte_data(self.i2c_address, base_reg + 2, off_time & 0xFF)
            self.bus.write_byte_data(self.i2c_address, base_reg + 3, off_time >> 8)
        except Exception as e:
            print(f"❌ Ошибка PWM: {e}")
    
    def rotate_motor(self, speed):
        """
        Вращение мотора
        speed: -100 до +100 (отрицательный = назад, положительный = вперед, 0 = стоп)
        """
        speed = max(-100, min(100, speed))
        
        # Преобразование скорости в PWM
        center_pulse = 307  # 1.5ms (стоп)
        max_range = 102     # Диапазон для полной скорости
        
        pulse_width = center_pulse + (speed / 100.0) * max_range
        self.set_pwm(0, int(pulse_width))
        
        if speed == 0:
            print("🛑 Мотор остановлен")
        elif speed > 0:
            print(f"➡️  Мотор вперед: {speed}%")
        else:
            print(f"⬅️  Мотор назад: {abs(speed)}%")
    
    def stop_motor(self):
        """Остановка мотора"""
        self.set_pwm(0, 0)
        print("🛑 Мотор остановлен")

class GPIOServoDriver:
    def __init__(self, pin1=4, pin2=12, channel=0):
        """
        Серводрайвер через GPIO
        
        Подключение:
        Raspberry Pi 5 → Серводрайвер:
        • 5V → VCC
        • GND → GND
        • GPIO 4 → Управление 1
        • GPIO 12 → Управление 2
        • Паурбанк → Питание моторов
        
        Мотор → Канал 0 драйвера
        """
        self.pin1 = pin1
        self.pin2 = pin2
        self.channel = channel
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        
        # PWM на одном из пинов для серво
        self.pwm = GPIO.PWM(self.pin1, 50)  # 50Hz
        self.pwm.start(0)
        
        print(f"🤖 GPIO серводрайвер: пины {pin1}, {pin2}, канал {channel}")
    
    def rotate_motor_angle(self, angle):
        """
        Поворот серво на угол (0-180°)
        """
        angle = max(0, min(180, angle))
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        
        self.pwm.ChangeDutyCycle(duty_cycle)
        print(f"🎯 Мотор на канале {self.channel}: {angle}°")
        time.sleep(0.5)
    
    def rotate_motor_direction(self, direction="forward", duration=2):
        """
        Вращение мотора в направлении
        """
        if direction == "forward":
            print(f"➡️  Мотор канал {self.channel} вперед ({duration}с)")
            GPIO.output(self.pin1, GPIO.HIGH)
            GPIO.output(self.pin2, GPIO.LOW)
        else:
            print(f"⬅️  Мотор канал {self.channel} назад ({duration}с)")
            GPIO.output(self.pin1, GPIO.LOW)
            GPIO.output(self.pin2, GPIO.HIGH)
        
        time.sleep(duration)
        self.stop_motor()
    
    def stop_motor(self):
        """Остановка мотора"""
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        print(f"🛑 Мотор канал {self.channel} остановлен")
    
    def cleanup(self):
        """Очистка GPIO"""
        self.pwm.stop()
        GPIO.cleanup()
        print("✅ GPIO очищены")

def test_i2c_driver():
    """Тест I2C драйвера"""
    if not I2C_AVAILABLE:
        print("❌ I2C недоступен")
        return
    
    try:
        print("\n🔧 Тест I2C серводрайвера...")
        driver = I2CServoDriver(channel=0)
        
        # Тест вращения
        speeds = [50, 0, -50, 0]
        for speed in speeds:
            driver.rotate_motor(speed)
            time.sleep(3)
        
        driver.stop_motor()
        print("✅ I2C тест завершен")
        
    except Exception as e:
        print(f"❌ I2C тест не удался: {e}")

def test_gpio_driver():
    """Тест GPIO драйвера"""
    try:
        print("\n🔧 Тест GPIO серводрайвера...")
        driver = GPIOServoDriver(pin1=4, pin2=12, channel=0)
        
        # Тест углов серво
        print("1️⃣  Тест позиций серво:")
        angles = [0, 90, 180, 90]
        for angle in angles:
            driver.rotate_motor_angle(angle)
            time.sleep(2)
        
        # Тест направлений
        print("2️⃣  Тест направлений:")
        driver.rotate_motor_direction("forward", 3)
        time.sleep(1)
        driver.rotate_motor_direction("backward", 3)
        
        driver.cleanup()
        print("✅ GPIO тест завершен")
        
    except Exception as e:
        print(f"❌ GPIO тест не удался: {e}")
        try:
            GPIO.cleanup()
        except:
            pass

def scan_i2c():
    """Поиск I2C устройств"""
    if not I2C_AVAILABLE:
        print("❌ I2C недоступен")
        return
    
    print("\n🔍 Поиск I2C устройств...")
    bus = smbus.SMBus(1)
    devices = []
    
    for addr in range(0x08, 0x78):
        try:
            bus.read_byte(addr)
            devices.append(addr)
            print(f"   Найдено устройство: 0x{addr:02x}")
        except:
            pass
    
    if not devices:
        print("❌ I2C устройства не найдены")
        print("💡 Проверьте подключение и включите I2C")
    else:
        print(f"✅ Найдено {len(devices)} устройств")

def simple_motor_control():
    """Простое управление мотором"""
    print("\n🎮 ПРОСТОЕ УПРАВЛЕНИЕ МОТОРОМ")
    print("="*40)
    
    # Пробуем сначала GPIO (проще)
    driver = GPIOServoDriver(pin1=4, pin2=12, channel=0)
    
    try:
        print("Команды:")
        print("1 - Вперед")
        print("2 - Назад") 
        print("3 - Позиции серво (0°, 90°, 180°)")
        print("0 - Стоп")
        print("x - Выход")
        
        while True:
            cmd = input("\nКоманда: ").strip()
            
            if cmd == "1":
                driver.rotate_motor_direction("forward", 2)
            elif cmd == "2":
                driver.rotate_motor_direction("backward", 2)
            elif cmd == "3":
                for angle in [0, 90, 180]:
                    driver.rotate_motor_angle(angle)
                    time.sleep(2)
            elif cmd == "0":
                driver.stop_motor()
            elif cmd == "x":
                break
            else:
                print("❓ Используйте: 1, 2, 3, 0, x")
    
    except KeyboardInterrupt:
        print("\n⚠️  Остановлено")
    finally:
        driver.cleanup()

def main():
    """Главная функция"""
    print("🤖 УПРАВЛЕНИЕ МОТОРОМ ЧЕРЕЗ СЕРВОДРАЙВЕР")
    print("="*50)
    print("📌 Raspberry Pi 5 → драйвер: 5V, GND, GPIO4, GPIO12")
    print("📌 Мотор → драйвер канал 0")
    print("📌 Паурбанк → питание драйвера")
    
    choice = input("""
Выберите режим:
1 - Тест I2C драйвера (PCA9685)
2 - Тест GPIO драйвера
3 - Поиск I2C устройств
4 - Простое управление
5 - Оба теста

Введите номер (1-5): """)
    
    try:
        if choice == "1":
            test_i2c_driver()
        elif choice == "2":
            test_gpio_driver()
        elif choice == "3":
            scan_i2c()
        elif choice == "4":
            simple_motor_control()
        elif choice == "5":
            print("🔄 Запуск всех тестов...")
            scan_i2c()
            time.sleep(2)
            test_i2c_driver()
            time.sleep(2) 
            test_gpio_driver()
        else:
            print("❌ Неверный выбор, запускаю простое управление")
            simple_motor_control()
    
    except KeyboardInterrupt:
        print("\n⚠️  Программа остановлена")
    except Exception as e:
        print(f"❌ Ошибка: {e}")
    finally:
        try:
            GPIO.cleanup()
        except:
            pass

if __name__ == "__main__":
    main()
    
    print("\n💡 Если мотор не крутится:")
    print("1. Проверьте питание от паурбанка")
    print("2. Убедитесь что мотор на канале 0")
    print("3. Для I2C: sudo raspi-config → Interface → I2C → Enable")
    print("4. Проверьте подключение проводов")