import RPi.GPIO as GPIO
import time

class ServoMecatronicos:
    def __init__(self, pin=3):
        """
        Аналог Arduino Servo библиотеки
        pin - GPIO пин для управления серво
        """
        # Переводим Arduino пин в GPIO (если нужно)
        # Arduino pin 3 обычно соответствует GPIO 17 на Raspberry Pi
        if pin == 3:
            self.gpio_pin = 17  # или другой GPIO по вашему подключению
        else:
            self.gpio_pin = pin
            
        print(f"🤖 Servo подключен к GPIO {self.gpio_pin}")
        
        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin, GPIO.OUT)
        
        # PWM с частотой 50Hz (стандарт для серво)
        self.pwm = GPIO.PWM(self.gpio_pin, 50)
        self.pwm.start(0)
        
    def attach(self, pin):
        """
        Аналог servomecatronicos.attach(pin)
        Уже выполнено в __init__
        """
        print(f"📌 Servo attached to pin {pin}")
        
    def write(self, angle):
        """
        Аналог servomecatronicos.write(angle)
        Поворот серво на указанный угол (0-180°)
        """
        # Ограничиваем угол
        angle = max(0, min(180, angle))
        
        # Преобразуем угол в duty cycle
        # 0° = 2.5%, 90° = 7.5%, 180° = 12.5%
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        
        # Устанавливаем позицию
        self.pwm.ChangeDutyCycle(duty_cycle)
        
        print(f"🎯 Servo position: {angle}°")
        
    def cleanup(self):
        """
        Очистка ресурсов
        """
        self.pwm.stop()
        GPIO.cleanup()
        print("✅ Servo cleanup completed")

# Создаем объект серво (аналог Servo servomecatronicos;)
servomecatronicos = ServoMecatronicos()

def setup():
    """
    Аналог void setup() в Arduino
    """
    print("🚀 Setup started...")
    servomecatronicos.attach(3)  # Подключаем к пину 3 (GPIO 17)
    print("✅ Setup completed!")

def loop():
    """
    Аналог void loop() в Arduino
    Выполняется один раз (не бесконечный цикл)
    """
    print("🔄 Loop started...")
    
    # delay(2000);
    print("⏱️  Delay 2 seconds...")
    time.sleep(2)
    
    # servomecatronicos.write(90);
    servomecatronicos.write(90)
    
    # delay(2000);
    print("⏱️  Delay 2 seconds...")
    time.sleep(2)
    
    # servomecatronicos.write(180);
    servomecatronicos.write(180)
    
    # delay(2000);
    print("⏱️  Delay 2 seconds...")
    time.sleep(2)
    
    # servomecatronicos.write(0);
    servomecatronicos.write(0)
    
    print("✅ Loop completed!")

def main():
    """
    Основная функция - запускает setup() и loop()
    """
    try:
        # Выполняем setup (как в Arduino)
        setup()
        
        # Выполняем loop несколько раз (или бесконечно)
        print("\n" + "="*50)
        print("🎮 Запуск Arduino-стиль программы...")
        print("="*50)
        
        # Вариант 1: Один цикл (как в оригинале)
        loop()
        
        # Вариант 2: Бесконечный цикл (раскомментируйте если нужно)
        # while True:
        #     loop()
        #     time.sleep(1)  # Небольшая пауза между циклами
        
    except KeyboardInterrupt:
        print("\n⚠️  Программа остановлена пользователем")
    
    finally:
        servomecatronicos.cleanup()

# Альтернативная версия с бесконечным циклом
def main_infinite():
    """
    Версия с бесконечным циклом (как настоящий Arduino)
    """
    try:
        setup()
        
        print("\n🔄 Запуск бесконечного цикла (Ctrl+C для остановки)...")
        
        while True:
            loop()
            time.sleep(0.1)  # Короткая пауза
            
    except KeyboardInterrupt:
        print("\n⚠️  Программа остановлена")
    finally:
        servomecatronicos.cleanup()

# Прямой перевод в функциональном стиле
def arduino_style_direct():
    """
    Максимально точный перевод Arduino кода
    """
    # Глобальная переменная (как в Arduino)
    global servomecatronicos
    
    try:
        print("🎯 Прямой перевод Arduino кода:")
        
        # setup()
        servomecatronicos.attach(3)
        
        # loop() - один раз
        time.sleep(2)                    # delay(2000);
        servomecatronicos.write(90)      # servomecatronicos.write(90);
        time.sleep(2)                    # delay(2000);
        servomecatronicos.write(180)     # servomecatronicos.write(180);
        time.sleep(2)                    # delay(2000);
        servomecatronicos.write(0)       # servomecatronicos.write(0);
        
        print("✅ Перевод выполнен!")
        
    except Exception as e:
        print(f"❌ Ошибка: {e}")
    finally:
        servomecatronicos.cleanup()

if __name__ == "__main__":
    print("🤖 ПЕРЕВОД ARDUINO КОДА НА PYTHON")
    print("="*50)
    
    choice = input("""
Выберите вариант запуска:
1 - Один цикл (как в оригинале)
2 - Бесконечный цикл (настоящий Arduino стиль)
3 - Прямой перевод кода

Введите номер (1-3): """)
    
    if choice == "1":
        main()
    elif choice == "2":
        main_infinite()
    elif choice == "3":
        arduino_style_direct()
    else:
        print("❌ Неверный выбор, запускаю вариант 1")
        main()

# Быстрый запуск
def quick_arduino_test():
    """Быстрый тест Arduino кода"""
    servo = ServoMecatronicos(pin=17)  # Используем GPIO 17
    try:
        servo.attach(3)
        
        # Последовательность из Arduino
        positions = [90, 180, 0]
        for pos in positions:
            time.sleep(2)
            servo.write(pos)
            
    finally:
        servo.cleanup()

# Использование:
# quick_arduino_test()  # Быстрый тест