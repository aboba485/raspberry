import RPi.GPIO as GPIO
import time
import threading

class MotorController:
    def __init__(self, pin1=4, pin2=5):
        """
        Инициализация контроллера мотора
        pin1, pin2 - пины управления направлением
        """
        self.pin1 = pin1
        self.pin2 = pin2
        self.is_running = False
        self.motor_thread = None
        
        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        
        # Инициализация PWM для управления скоростью
        self.pwm1 = GPIO.PWM(self.pin1, 1000)  # 1000 Hz
        self.pwm2 = GPIO.PWM(self.pin2, 1000)  # 1000 Hz
        self.pwm1.start(0)
        self.pwm2.start(0)
        
        print("🤖 Контроллер мотора инициализирован")
    
    def forward(self, speed=100):
        """
        Вращение вперед
        speed: скорость 0-100%
        """
        print(f"➡️  Вращение вперед, скорость: {speed}%")
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(0)
    
    def backward(self, speed=100):
        """
        Вращение назад
        speed: скорость 0-100%
        """
        print(f"⬅️  Вращение назад, скорость: {speed}%")
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(speed)
    
    def stop(self):
        """
        Остановка мотора
        """
        print("🛑 Мотор остановлен")
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        self.is_running = False
    
    def spin_continuous(self, direction="forward", speed=100):
        """
        Непрерывное вращение в отдельном потоке
        direction: "forward" или "backward"
        speed: скорость 0-100%
        """
        if self.is_running:
            print("⚠️  Мотор уже работает! Остановите его сначала.")
            return
        
        self.is_running = True
        
        def spin():
            print(f"🔄 Начинаю непрерывное вращение ({direction}, {speed}%)")
            while self.is_running:
                if direction == "forward":
                    self.forward(speed)
                else:
                    self.backward(speed)
                time.sleep(0.1)  # Небольшая задержка
        
        self.motor_thread = threading.Thread(target=spin)
        self.motor_thread.daemon = True
        self.motor_thread.start()
    
    def change_speed(self, new_speed):
        """
        Изменение скорости во время работы
        """
        if self.is_running:
            print(f"⚡ Изменение скорости на {new_speed}%")
            # Определяем текущее направление по активному PWM
            if self.pwm1.duty_cycle > 0:
                self.pwm1.ChangeDutyCycle(new_speed)
            elif self.pwm2.duty_cycle > 0:
                self.pwm2.ChangeDutyCycle(new_speed)
        else:
            print("⚠️  Мотор не работает")
    
    def reverse_direction(self):
        """
        Смена направления вращения
        """
        if self.is_running:
            print("🔄 Смена направления")
            # Сохраняем текущую скорость
            current_speed = max(self.pwm1.duty_cycle, self.pwm2.duty_cycle)
            
            # Меняем направление
            if self.pwm1.duty_cycle > 0:
                self.backward(current_speed)
            else:
                self.forward(current_speed)
        else:
            print("⚠️  Мотор не работает")
    
    def cleanup(self):
        """
        Очистка ресурсов
        """
        self.stop()
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.cleanup()
        print("✅ Ресурсы очищены")

# Демонстрация использования
if __name__ == "__main__":
    # Создаем контроллер мотора
    motor = MotorController()
    
    try:
        print("\n=== ДЕМОНСТРАЦИЯ РАБОТЫ МОТОРА ===")
        
        # Тест 1: Вращение вперед с разными скоростями
        print("\n1️⃣  Тест скоростей (вперед)")
        for speed in [30, 60, 100]:
            motor.forward(speed)
            time.sleep(2)
        
        motor.stop()
        time.sleep(1)
        
        # Тест 2: Вращение назад
        print("\n2️⃣  Тест назад")
        motor.backward(80)
        time.sleep(3)
        
        motor.stop()
        time.sleep(1)
        
        # Тест 3: Непрерывное вращение в фоне
        print("\n3️⃣  Непрерывное вращение (10 секунд)")
        motor.spin_continuous("forward", 70)
        
        # Через 3 секунды меняем скорость
        time.sleep(3)
        motor.change_speed(100)
        
        # Через 3 секунды меняем направление
        time.sleep(3)
        motor.reverse_direction()
        
        # Ещё 4 секунды и останавливаем
        time.sleep(4)
        motor.stop()
        
        print("\n✅ Демонстрация завершена!")
        
    except KeyboardInterrupt:
        print("\n⚠️  Прерывание пользователем")
    
    finally:
        motor.cleanup()

# Простая функция для быстрого запуска
def quick_spin(direction="forward", speed=100, duration=5):
    """
    Быстрый запуск мотора
    direction: "forward" или "backward"
    speed: скорость 0-100%
    duration: время в секундах
    """
    motor = MotorController()
    try:
        print(f"🚀 Быстрый старт: {direction}, {speed}%, {duration}с")
        
        if direction == "forward":
            motor.forward(speed)
        else:
            motor.backward(speed)
        
        time.sleep(duration)
        
    finally:
        motor.cleanup()

# Пример использования:
# quick_spin("forward", 80, 10)  # Вперед, 80%, 10 секунд