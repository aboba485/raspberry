import RPi.GPIO as GPIO
import time
import threading

class ServoDriverController:
    def __init__(self, control_pin=15):
        """
        Управление мотором через серво драйвер
        control_pin - пин управления серво драйвером
        
        Подключение:
        Raspberry Pi → Серво Драйвер:
        • 5V → VCC 
        • GND → GND
        • GPIO 4 → Управление драйвером (возможно)
        • GPIO 5 → Управление драйвером (возможно)
        
        Серво Драйвер → Мотор:
        • Выход 15 → Мотор
        """
        self.control_pin = control_pin
        self.current_position = 90  # Средняя позиция
        self.is_running = False
        self.motor_thread = None
        
        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.control_pin, GPIO.OUT)
        
        # PWM для серво драйвера (50Hz стандарт)
        self.pwm = GPIO.PWM(self.control_pin, 50)
        self.pwm.start(0)
        
        print("🤖 Серво драйвер контроллер инициализирован")
        print(f"📌 Управляющий пин: {self.control_pin}")
        
        # Установка начальной позиции
        self.set_position(90)
        time.sleep(1)
    
    def angle_to_duty_cycle(self, angle):
        """
        Преобразование угла в duty cycle
        """
        angle = max(0, min(180, angle))
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        return duty_cycle
    
    def set_position(self, angle):
        """
        Установка позиции мотора
        """
        angle = max(0, min(180, angle))
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.current_position = angle
        print(f"🎯 Позиция: {angle}°")
        time.sleep(0.5)
    
    def rotate_continuous(self, direction="forward", speed=2):
        """
        Непрерывное вращение
        """
        if self.is_running:
            print("⚠️  Мотор уже работает!")
            return
        
        self.is_running = True
        
        def continuous_rotation():
            print(f"🔄 Непрерывное вращение ({direction})")
            angle = self.current_position
            
            while self.is_running:
                if direction == "forward":
                    angle += 5
                    if angle > 180:
                        angle = 0
                else:
                    angle -= 5
                    if angle < 0:
                        angle = 180
                
                duty_cycle = self.angle_to_duty_cycle(angle)
                self.pwm.ChangeDutyCycle(duty_cycle)
                self.current_position = angle
                time.sleep(0.1 / speed)
        
        self.motor_thread = threading.Thread(target=continuous_rotation)
        self.motor_thread.daemon = True
        self.motor_thread.start()
    
    def stop(self):
        """
        Остановка мотора
        """
        print("🛑 Остановка мотора")
        self.is_running = False
        self.pwm.ChangeDutyCycle(0)  # Отключаем сигнал
        time.sleep(0.5)
    
    def test_basic_positions(self):
        """
        Тест базовых позиций
        """
        print("🔧 Тест базовых позиций...")
        positions = [0, 45, 90, 135, 180, 90]
        
        for pos in positions:
            print(f"📍 Позиция {pos}°")
            self.set_position(pos)
            time.sleep(1)
    
    def cleanup(self):
        """
        Очистка ресурсов
        """
        self.stop()
        self.pwm.stop()
        GPIO.cleanup()
        print("✅ Ресурсы очищены")

# АЛЬТЕРНАТИВНЫЙ КОНТРОЛЛЕР для случая, если это DC мотор через драйвер
class DCMotorThroughDriver:
    def __init__(self, pin1=4, pin2=5):
        """
        DC мотор через драйвер с управлением по GPIO 4 и 5
        """
        self.pin1 = pin1
        self.pin2 = pin2
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        
        # Останавливаем мотор
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)
        
        print("🤖 DC мотор через драйвер инициализирован")
        print(f"📌 Пины управления: {self.pin1}, {self.pin2}")
    
    def forward(self, duration=None):
        """Вращение вперед"""
        print("➡️  Мотор вперед")
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW)
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def backward(self, duration=None):
        """Вращение назад"""
        print("⬅️  Мотор назад")
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.HIGH)
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def stop(self):
        """Остановка"""
        print("🛑 Мотор остановлен")
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)
    
    def test_motor(self):
        """Базовый тест"""
        print("🔧 Тест DC мотора...")
        
        print("1️⃣  Вперед 2 сек")
        self.forward(2)
        time.sleep(0.5)
        
        print("2️⃣  Назад 2 сек")
        self.backward(2)
        time.sleep(0.5)
        
        print("✅ Тест завершен")
    
    def cleanup(self):
        """Очистка"""
        self.stop()
        GPIO.cleanup()
        print("✅ GPIO очищены")

# ДИАГНОСТИЧЕСКАЯ ФУНКЦИЯ
def diagnose_setup():
    """
    Диагностика подключения
    """
    print("🔍 ДИАГНОСТИКА ПОДКЛЮЧЕНИЯ")
    print("=" * 40)
    
    # Тест 1: Серво управление через пин 15
    print("\n1️⃣  Тест серво управления (GPIO 15)")
    try:
        servo = ServoDriverController(control_pin=15)
        servo.test_basic_positions()
        servo.cleanup()
        print("✅ Серво тест завершен")
    except Exception as e:
        print(f"❌ Ошибка серво: {e}")
    
    time.sleep(2)
    
    # Тест 2: DC мотор через драйвер (GPIO 4,5)
    print("\n2️⃣  Тест DC мотора (GPIO 4,5)")
    try:
        dc_motor = DCMotorThroughDriver(pin1=4, pin2=5)
        dc_motor.test_motor()
        dc_motor.cleanup()
        print("✅ DC мотор тест завершен")
    except Exception as e:
        print(f"❌ Ошибка DC: {e}")

# Основная демонстрация
if __name__ == "__main__":
    print("🤖 КОНТРОЛЛЕР МОТОРА ЧЕРЕЗ СЕРВО ДРАЙВЕР")
    print("=" * 50)
    
    choice = input("""
Выберите тип управления:
1 - Серво управление (GPIO 15)
2 - DC мотор (GPIO 4,5) 
3 - Диагностика всех вариантов

Введите номер (1-3): """)
    
    try:
        if choice == "1":
            # Серво управление
            controller = ServoDriverController(control_pin=15)
            
            print("\n=== ДЕМО СЕРВО УПРАВЛЕНИЯ ===")
            controller.test_basic_positions()
            
            print("\n🔄 Непрерывное вращение (5 сек)")
            controller.rotate_continuous("forward", speed=3)
            time.sleep(5)
            controller.stop()
            
            controller.cleanup()
            
        elif choice == "2":
            # DC мотор
            controller = DCMotorThroughDriver()
            
            print("\n=== ДЕМО DC МОТОРА ===")
            controller.test_motor()
            
            print("\n🔄 Длительное вращение")
            controller.forward(3)
            controller.backward(3)
            
            controller.cleanup()
            
        elif choice == "3":
            # Диагностика
            diagnose_setup()
            
        else:
            print("❌ Неверный выбор")
            
    except KeyboardInterrupt:
        print("\n⚠️  Остановлено пользователем")
    except Exception as e:
        print(f"❌ Ошибка: {e}")
    finally:
        try:
            GPIO.cleanup()
        except:
            pass

# Быстрые функции
def quick_servo_test():
    """Быстрый тест серво"""
    controller = ServoDriverController(15)
    try:
        controller.test_basic_positions()
    finally:
        controller.cleanup()

def quick_dc_test():
    """Быстрый тест DC"""
    controller = DCMotorThroughDriver()
    try:
        controller.test_motor()
    finally:
        controller.cleanup()