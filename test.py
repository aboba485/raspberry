import RPi.GPIO as GPIO
import time
import threading

class ServoMotor:
    def __init__(self, servo_pin=13):
        """
        Управление серво мотором на GPIO 13
        
        Подключение:
        • Красный (VCC) → 5V Raspberry Pi
        • Черный/Коричневый (GND) → Ground Raspberry Pi  
        • Желтый/Оранжевый (Signal) → GPIO 13
        """
        self.servo_pin = servo_pin
        self.current_angle = 90
        self.is_running = False
        self.rotation_thread = None
        
        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # PWM 50Hz для серво
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
        
        print(f"🤖 Серво мотор подключен к GPIO {self.servo_pin}")
        
        # Устанавливаем начальную позицию
        self.move_to(90)
        time.sleep(1)
    
    def angle_to_duty_cycle(self, angle):
        """
        Преобразование угла в duty cycle для PWM
        0° = 2.5%, 90° = 7.5%, 180° = 12.5%
        """
        angle = max(0, min(180, angle))
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        return duty_cycle
    
    def move_to(self, angle):
        """
        Поворот серво на указанный угол (0-180°)
        """
        angle = max(0, min(180, angle))
        duty_cycle = self.angle_to_duty_cycle(angle)
        
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.current_angle = angle
        
        print(f"🎯 Серво: {angle}°")
        time.sleep(0.5)  # Время для поворота
        
        # Останавливаем PWM после поворота (экономия энергии)
        self.pwm.ChangeDutyCycle(0)
    
    def sweep(self, start=0, end=180, step=5, delay=0.1):
        """
        Плавное качание между углами
        """
        print(f"🔄 Качание от {start}° до {end}°")
        
        # Туда
        for angle in range(start, end + 1, step):
            self.move_to(angle)
            time.sleep(delay)
        
        # Обратно
        for angle in range(end, start - 1, -step):
            self.move_to(angle)
            time.sleep(delay)
    
    def continuous_rotation(self, speed=1):
        """
        Имитация непрерывного вращения
        (для обычных серво - поворот от 0 до 180 и обратно)
        """
        if self.is_running:
            print("⚠️  Серво уже работает!")
            return
        
        self.is_running = True
        
        def rotate():
            print("🔄 Начинаю непрерывное вращение")
            while self.is_running:
                # Полный оборот: 0° → 180° → 0°
                for angle in range(0, 181, 5):
                    if not self.is_running:
                        break
                    self.move_to(angle)
                    time.sleep(0.05 / speed)
                
                for angle in range(180, -1, -5):
                    if not self.is_running:
                        break
                    self.move_to(angle)
                    time.sleep(0.05 / speed)
        
        self.rotation_thread = threading.Thread(target=rotate)
        self.rotation_thread.daemon = True
        self.rotation_thread.start()
    
    def stop_rotation(self):
        """
        Остановка непрерывного вращения
        """
        if self.is_running:
            print("🛑 Остановка вращения")
            self.is_running = False
            if self.rotation_thread:
                self.rotation_thread.join()
    
    def center(self):
        """
        Возврат в центральное положение
        """
        print("🎯 Возврат в центр")
        self.move_to(90)
    
    def test_basic_positions(self):
        """
        Тест основных позиций
        """
        print("🔧 Тест основных позиций...")
        positions = [0, 45, 90, 135, 180, 90]
        
        for pos in positions:
            print(f"📍 Позиция: {pos}°")
            self.move_to(pos)
            time.sleep(1)
    
    def arduino_sequence(self):
        """
        Последовательность из Arduino кода
        """
        print("🎮 Выполнение Arduino последовательности...")
        
        time.sleep(2)
        self.move_to(90)
        
        time.sleep(2)
        self.move_to(180)
        
        time.sleep(2)
        self.move_to(0)
        
        print("✅ Arduino последовательность завершена")
    
    def cleanup(self):
        """
        Очистка ресурсов
        """
        self.stop_rotation()
        self.pwm.stop()
        GPIO.cleanup()
        print("✅ Серво отключен")

# Быстрые функции
def quick_test():
    """
    Быстрый тест серво
    """
    servo = ServoMotor(servo_pin=13)
    
    try:
        print("🚀 БЫСТРЫЙ ТЕСТ СЕРВО")
        servo.test_basic_positions()
        
    except KeyboardInterrupt:
        print("\n⚠️  Тест прерван")
    finally:
        servo.cleanup()

def arduino_style():
    """
    Выполнение в стиле Arduino
    """
    servo = ServoMotor(servo_pin=13)
    
    try:
        print("🎮 ARDUINO СТИЛЬ")
        
        # Бесконечный цикл как в Arduino
        while True:
            servo.arduino_sequence()
            time.sleep(1)  # Пауза между циклами
            
    except KeyboardInterrupt:
        print("\n⚠️  Остановлено пользователем")
    finally:
        servo.cleanup()

def continuous_spin():
    """
    Непрерывное вращение
    """
    servo = ServoMotor(servo_pin=13)
    
    try:
        print("🔄 НЕПРЕРЫВНОЕ ВРАЩЕНИЕ")
        servo.continuous_rotation(speed=2)
        
        # Крутим 10 секунд
        time.sleep(10)
        
    except KeyboardInterrupt:
        print("\n⚠️  Остановлено пользователем")
    finally:
        servo.cleanup()

# Основная демонстрация
if __name__ == "__main__":
    print("🤖 УПРАВЛЕНИЕ СЕРВО МОТОРОМ GPIO 13")
    print("=" * 50)
    print("📌 Подключение:")
    print("   Красный → 5V")
    print("   Черный → GND")  
    print("   Желтый → GPIO 13")
    print("=" * 50)
    
    choice = input("""
Выберите режим:
1 - Быстрый тест позиций
2 - Arduino стиль (бесконечный цикл)
3 - Непрерывное вращение
4 - Ручное управление
5 - Качание

Введите номер (1-5): """)
    
    try:
        if choice == "1":
            quick_test()
            
        elif choice == "2":
            arduino_style()
            
        elif choice == "3":
            continuous_spin()
            
        elif choice == "4":
            # Ручное управление
            servo = ServoMotor(servo_pin=13)
            try:
                print("\n🎮 РУЧНОЕ УПРАВЛЕНИЕ")
                print("Введите угол (0-180) или 'q' для выхода:")
                
                while True:
                    user_input = input("Угол: ").strip()
                    
                    if user_input.lower() == 'q':
                        break
                    
                    try:
                        angle = int(user_input)
                        servo.move_to(angle)
                    except ValueError:
                        print("❌ Введите число от 0 до 180")
                        
            finally:
                servo.cleanup()
                
        elif choice == "5":
            # Качание
            servo = ServoMotor(servo_pin=13)
            try:
                print("\n🔄 РЕЖИМ КАЧАНИЯ")
                servo.sweep(start=30, end=150, step=3, delay=0.05)
                
            finally:
                servo.cleanup()
                
        else:
            print("❌ Неверный выбор, запускаю быстрый тест")
            quick_test()
            
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        try:
            GPIO.cleanup()
        except:
            pass

# Простая функция для тестирования
def simple_move(angle):
    """
    Простой поворот на угол
    """
    servo = ServoMotor(13)
    try:
        servo.move_to(angle)
        time.sleep(2)
    finally:
        servo.cleanup()

# Примеры использования:
# simple_move(90)  # Поворот на 90°
# quick_test()     # Быстрый тест
# arduino_style()  # Как в Arduino