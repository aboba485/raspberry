import RPi.GPIO as GPIO
import time
import sys
import select
import tty
import termios

class SimpleServo:
    def __init__(self, gpio_pin=12):
        """
        Простой серво контроллер
        """
        self.gpio_pin = gpio_pin
        self.current_angle = 90
        
        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.OUT)
        
        # PWM 50Hz
        self.pwm = GPIO.PWM(self.gpio_pin, 50)
        self.pwm.start(0)
        
        print(f"🤖 Серво готово на GPIO {self.gpio_pin}")
        
        # Начальная позиция
        self.move_to(90)
    
    def move_to(self, angle):
        """
        Поворот на угол
        """
        angle = max(0, min(180, angle))
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.current_angle = angle
        
        print(f"🎯 Угол: {angle}°")
        time.sleep(0.3)  # Короткая пауза
    
    def turn_right(self, step=10):
        """
        Поворот вправо
        """
        new_angle = min(180, self.current_angle + step)
        print("➡️  Вправо")
        self.move_to(new_angle)
    
    def turn_left(self, step=10):
        """
        Поворот влево  
        """
        new_angle = max(0, self.current_angle - step)
        print("⬅️  Влево")
        self.move_to(new_angle)
    
    def center(self):
        """
        В центр
        """
        print("🎯 Центр")
        self.move_to(90)
    
    def cleanup(self):
        """
        Очистка
        """
        self.pwm.stop()
        GPIO.cleanup()
        print("✅ Очищено")

def get_char():
    """
    Получение одного символа без Enter
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.cbreak(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def keyboard_control():
    """
    Управление с клавиатуры
    """
    servo = SimpleServo(gpio_pin=12)
    
    print("\n" + "="*50)
    print("🎮 УПРАВЛЕНИЕ СЕРВО С КЛАВИАТУРЫ")
    print("="*50)
    print("🔴 Q - ПОВОРОТ ВПРАВО")
    print("🔵 E - ПОВОРОТ ВЛЕВО") 
    print("🟡 S - ЦЕНТР")
    print("🟢 X - ВЫХОД")
    print("="*50)
    print("Нажимайте клавиши (без Enter):")
    
    try:
        while True:
            key = get_char().lower()
            
            if key == 'q':
                servo.turn_right()
            elif key == 'e':
                servo.turn_left()
            elif key == 's':
                servo.center()
            elif key == 'x':
                print("👋 Выход...")
                break
            elif key == '\x03':  # Ctrl+C
                break
            else:
                print(f"❓ Неизвестная клавиша: {key}")
                
    except KeyboardInterrupt:
        print("\n⚠️  Остановлено")
    finally:
        servo.cleanup()

def simple_control():
    """
    Упрощенное управление для терминалов без tty
    """
    servo = SimpleServo(gpio_pin=12)
    
    print("\n" + "="*50)
    print("🎮 ПРОСТОЕ УПРАВЛЕНИЕ СЕРВО")
    print("="*50)
    print("Команды:")
    print("q + Enter - ВПРАВО")
    print("e + Enter - ВЛЕВО")
    print("s + Enter - ЦЕНТР") 
    print("x + Enter - ВЫХОД")
    print("="*50)
    
    try:
        while True:
            command = input("Команда: ").lower().strip()
            
            if command == 'q':
                servo.turn_right()
            elif command == 'e':
                servo.turn_left()
            elif command == 's':
                servo.center()
            elif command == 'x':
                print("👋 Выход...")
                break
            else:
                print("❓ Используйте: q, e, s, x")
                
    except KeyboardInterrupt:
        print("\n⚠️  Остановлено")
    finally:
        servo.cleanup()

def test_basic():
    """
    Базовый тест (работает лучше всех)
    """
    servo = SimpleServo(gpio_pin=12)
    
    try:
        print("🔧 Базовый тест...")
        
        angles = [90, 120, 90, 60, 90, 150, 90, 30, 90]
        
        for angle in angles:
            servo.move_to(angle)
            time.sleep(1)
        
        print("✅ Тест завершен")
        
    finally:
        servo.cleanup()

# Быстрые функции
def quick_right():
    """Быстрый поворот вправо"""
    servo = SimpleServo(12)
    try:
        servo.turn_right(30)
    finally:
        servo.cleanup()

def quick_left():
    """Быстрый поворот влево"""
    servo = SimpleServo(12)
    try:
        servo.turn_left(30)
    finally:
        servo.cleanup()

def quick_center():
    """Быстрый центр"""
    servo = SimpleServo(12)
    try:
        servo.center()
    finally:
        servo.cleanup()

# Основная программа
if __name__ == "__main__":
    print("🤖 ПРОСТОЕ УПРАВЛЕНИЕ СЕРВО")
    print("="*40)
    
    choice = input("""
Выберите режим:
1 - Управление клавишами (без Enter)
2 - Простое управление (с Enter)
3 - Базовый тест
4 - Быстрые команды

Введите номер (1-4): """)
    
    try:
        if choice == "1":
            keyboard_control()
        elif choice == "2":
            simple_control()
        elif choice == "3":
            test_basic()
        elif choice == "4":
            print("\nБыстрые команды:")
            print("quick_right()  - вправо")
            print("quick_left()   - влево") 
            print("quick_center() - центр")
            test_basic()
        else:
            print("❌ Неверный выбор, запускаю простое управление")
            simple_control()
            
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        print("Попробуйте режим 2 (простое управление)")
        simple_control()

# ИНСТРУКЦИЯ:
"""
🎮 КАК ИСПОЛЬЗОВАТЬ:

Вариант 1 - Клавиши без Enter:
python script.py -> выбрать 1 -> нажимать q/e/s/x

Вариант 2 - С Enter:  
python script.py -> выбрать 2 -> вводить q/e/s/x + Enter

Вариант 3 - Автотест:
python script.py -> выбрать 3

🔴 Q = ВПРАВО (увеличение угла)
🔵 E = ВЛЕВО (уменьшение угла)  
🟡 S = ЦЕНТР (90°)
🟢 X = ВЫХОД
"""