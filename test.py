import RPi.GPIO as GPIO
import time
import threading

class ServoMotor:
    def __init__(self, gpio_pin=12):
        """
        Серво мотор подключенный напрямую к Raspberry Pi 5
        
        Подключение:
        • Красный (VCC) → 5V Raspberry Pi
        • Черный/Коричневый (GND) → Ground Raspberry Pi  
        • Желтый/Оранжевый (Signal) → GPIO 12
        """
        self.gpio_pin = gpio_pin
        self.current_angle = 90
        self.is_sweeping = False
        self.sweep_thread = None
        
        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.OUT)
        
        # PWM с частотой 50Hz (стандарт для серво)
        self.pwm = GPIO.PWM(self.gpio_pin, 50)
        self.pwm.start(0)
        
        print(f"🤖 Серво мотор подключен к GPIO {self.gpio_pin}")
        
        # Устанавливаем начальную позицию
        self.write(90)
        time.sleep(1)
    
    def angle_to_duty_cycle(self, angle):
        """
        Преобразование угла в duty cycle для PWM
        0° = 2.5%, 90° = 7.5%, 180° = 12.5%
        """
        angle = max(0, min(180, angle))
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        return duty_cycle
    
    def write(self, angle):
        """
        Поворот серво на указанный угол (0-180°)
        """
        angle = max(0, min(180, angle))
        duty_cycle = self.angle_to_duty_cycle(angle)
        
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.current_angle = angle
        
        print(f"🎯 Серво повернуто на {angle}°")
        time.sleep(0.5)  # Время для поворота
    
    def sweep(self, start_angle=0, end_angle=180, step=5, delay=0.1):
        """
        Плавное качание между углами
        """
        print(f"🔄 Качание от {start_angle}° до {end_angle}°")
        
        # Движение к end_angle
        if start_angle < end_angle:
            for angle in range(start_angle, end_angle + 1, step):
                self.write(angle)
                time.sleep(delay)
        else:
            for angle in range(start_angle, end_angle - 1, -step):
                self.write(angle)
                time.sleep(delay)
    
    def continuous_sweep(self, start_angle=0, end_angle=180, speed=1):
        """
        Непрерывное качание в отдельном потоке
        """
        if self.is_sweeping:
            print("⚠️  Серво уже качается!")
            return
        
        self.is_sweeping = True
        
        def sweep_motion():
            print(f"🔄 Непрерывное качание: {start_angle}° ↔ {end_angle}°")
            
            while self.is_sweeping:
                # Движение вперед
                for angle in range(start_angle, end_angle + 1, 2):
                    if not self.is_sweeping:
                        break
                    self.write(angle)
                    time.sleep(0.05 / speed)
                
                # Движение назад
                for angle in range(end_angle, start_angle - 1, -2):
                    if not self.is_sweeping:
                        break
                    self.write(angle)
                    time.sleep(0.05 / speed)
        
        self.sweep_thread = threading.Thread(target=sweep_motion)
        self.sweep_thread.daemon = True
        self.sweep_thread.start()
    
    def rotate_continuous(self, direction="clockwise", speed=2):
        """
        Имитация непрерывного вращения
        """
        if self.is_sweeping:
            print("⚠️  Серво уже работает!")
            return
        
        self.is_sweeping = True
        
        def continuous_rotation():
            print(f"🔄 Непрерывное вращение ({direction})")
            angle = self.current_angle
            
            while self.is_sweeping:
                if direction == "clockwise":
                    angle += 10
                    if angle > 180:
                        angle = 0
                else:
                    angle -= 10
                    if angle < 0:
                        angle = 180
                
                self.write(angle)
                time.sleep(0.1 / speed)
        
        self.sweep_thread = threading.Thread(target=continuous_rotation)
        self.sweep_thread.daemon = True
        self.sweep_thread.start()
    
    def stop(self):
        """
        Остановка движения
        """
        if self.is_sweeping:
            print("🛑 Остановка серво")
            self.is_sweeping = False
            if self.sweep_thread:
                self.sweep_thread.join()
    
    def center(self):
        """
        Возврат в центральное положение (90°)
        """
        print("🎯 Возврат в центр")
        self.write(90)
    
    def test_basic_positions(self):
        """
        Тест основных позиций
        """
        print("🔧 Тест основных позиций...")
        positions = [0, 45, 90, 135, 180, 90]
        
        for pos in positions:
            print(f"📍 Позиция: {pos}°")
            self.write(pos)
            time.sleep(1)
        
        print("✅ Тест позиций завершен")
    
    def cleanup(self):
        """
        Очистка ресурсов
        """
        self.stop()
        self.pwm.stop()
        GPIO.cleanup()
        print("✅ Ресурсы очищены")

# Перевод Arduino кода
class ArduinoStyleServo:
    def __init__(self, pin=12):
        self.servo = ServoMotor(pin)
    
    def attach(self, pin):
        """Аналог servo.attach()"""
        print(f"📌 Servo attached to GPIO {pin}")
    
    def write(self, angle):
        """Аналог servo.write()"""
        self.servo.write(angle)
    
    def cleanup(self):
        self.servo.cleanup()

# Глобальный объект серво (как в Arduino)
servomecatronicos = ArduinoStyleServo(pin=12)

def setup():
    """Arduino setup()"""
    print("🚀 Setup...")
    servomecatronicos.attach(12)
    print("✅ Setup завершен!")

def loop():
    """Arduino loop() - один цикл"""
    print("🔄 Loop...")
    
    time.sleep(2)                    # delay(2000)
    servomecatronicos.write(90)      # servomecatronicos.write(90)
    
    time.sleep(2)                    # delay(2000)
    servomecatronicos.write(180)     # servomecatronicos.write(180)
    
    time.sleep(2)                    # delay(2000)
    servomecatronicos.write(0)       # servomecatronicos.write(0)
    
    print("✅ Loop завершен!")

# Демонстрационные функции
def demo_basic():
    """Базовая демонстрация"""
    servo = ServoMotor(gpio_pin=12)
    
    try:
        print("\n=== БАЗОВАЯ ДЕМОНСТРАЦИЯ ===")
        servo.test_basic_positions()
        
    finally:
        servo.cleanup()

def demo_sweeping():
    """Демонстрация качания"""
    servo = ServoMotor(gpio_pin=12)
    
    try:
        print("\n=== ДЕМОНСТРАЦИЯ КАЧАНИЯ ===")
        
        # Плавное качание
        print("1️⃣  Плавное качание")
        servo.sweep(30, 150, step=3, delay=0.05)
        
        time.sleep(1)
        
        # Непрерывное качание
        print("2️⃣  Непрерывное качание (5 сек)")
        servo.continuous_sweep(45, 135, speed=2)
        time.sleep(5)
        servo.stop()
        
        # Возврат в центр
        servo.center()
        
    finally:
        servo.cleanup()

def demo_continuous_rotation():
    """Демонстрация непрерывного вращения"""
    servo = ServoMotor(gpio_pin=12)
    
    try:
        print("\n=== НЕПРЕРЫВНОЕ ВРАЩЕНИЕ ===")
        
        print("🔄 Вращение по часовой (5 сек)")
        servo.rotate_continuous("clockwise", speed=3)
        time.sleep(5)
        
        print("🔄 Смена направления")
        servo.stop()
        time.sleep(0.5)
        
        servo.rotate_continuous("counterclockwise", speed=3)
        time.sleep(5)
        servo.stop()
        
        servo.center()
        
    finally:
        servo.cleanup()

def arduino_demo():
    """Точная копия Arduino кода"""
    try:
        print("\n=== ARDUINO ДЕМОНСТРАЦИЯ ===")
        
        setup()
        
        # Запускаем несколько циклов
        for i in range(3):
            print(f"\n--- Цикл {i+1} ---")
            loop()
        
    finally:
        servomecatronicos.cleanup()

# Основная программа
if __name__ == "__main__":
    print("🤖 СЕРВО МОТОР RASPBERRY PI 5")
    print("=" * 50)
    print("📌 Подключение: GPIO 12, 5V, GND")
    
    choice = input("""
Выберите демонстрацию:
1 - Базовые позиции
2 - Качание
3 - Непрерывное вращение  
4 - Arduino стиль код
5 - Все демонстрации

Введите номер (1-5): """)
    
    try:
        if choice == "1":
            demo_basic()
        elif choice == "2":
            demo_sweeping()
        elif choice == "3":
            demo_continuous_rotation()
        elif choice == "4":
            arduino_demo()
        elif choice == "5":
            print("🎬 Все демонстрации подряд:")
            demo_basic()
            time.sleep(2)
            demo_sweeping()
            time.sleep(2)
            demo_continuous_rotation()
            time.sleep(2)
            arduino_demo()
        else:
            print("❌ Неверный выбор, запускаю базовую демонстрацию")
            demo_basic()
            
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
def quick_test():
    """Быстрый тест серво"""
    servo = ServoMotor(12)
    try:
        servo.write(0)
        time.sleep(1)
        servo.write(90)
        time.sleep(1)
        servo.write(180)
        time.sleep(1)
        servo.write(90)
    finally:
        servo.cleanup()

def quick_arduino():
    """Быстрый Arduino тест"""
    try:
        setup()
        loop()
    finally:
        servomecatronicos.cleanup()

# Использование:
# quick_test()      # Быстрый тест
# quick_arduino()   # Arduino стиль