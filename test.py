import RPi.GPIO as GPIO
import time
import threading

class ServoController:
    def __init__(self, servo_pin=15):
        """
        Инициализация контроллера серво мотора
        servo_pin - пин управления серво (обычно 15)
        """
        self.servo_pin = servo_pin
        self.current_angle = 90  # Начальная позиция
        self.is_sweeping = False
        self.sweep_thread = None
        
        # Настройка GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # PWM с частотой 50Hz для серво
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
        
        print("🤖 Контроллер серво мотора инициализирован")
        
        # Устанавливаем начальную позицию
        self.move_to_angle(90)
        time.sleep(1)
    
    def angle_to_duty_cycle(self, angle):
        """
        Преобразование угла в duty cycle для PWM
        Стандартные серво: 0° = 2.5%, 90° = 7.5%, 180° = 12.5%
        """
        # Ограничиваем угол от 0 до 180
        angle = max(0, min(180, angle))
        
        # Преобразуем в duty cycle (2.5% до 12.5%)
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        return duty_cycle
    
    def move_to_angle(self, angle, smooth=True):
        """
        Поворот серво на указанный угол
        angle: угол от 0 до 180 градусов
        smooth: плавное движение
        """
        angle = max(0, min(180, angle))
        
        if smooth and abs(angle - self.current_angle) > 5:
            # Плавное движение
            step = 1 if angle > self.current_angle else -1
            for a in range(int(self.current_angle), int(angle), step):
                duty_cycle = self.angle_to_duty_cycle(a)
                self.pwm.ChangeDutyCycle(duty_cycle)
                time.sleep(0.02)  # 20ms задержка
        
        # Устанавливаем финальный угол
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.current_angle = angle
        
        print(f"🎯 Серво повернуто на {angle}°")
        time.sleep(0.5)  # Время для стабилизации
    
    def sweep(self, start_angle=0, end_angle=180, speed=1, continuous=False):
        """
        Качание серво между двумя углами
        start_angle, end_angle: диапазон углов
        speed: скорость (задержка между шагами)
        continuous: непрерывное качание
        """
        if self.is_sweeping:
            print("⚠️  Серво уже качается! Остановите его сначала.")
            return
        
        self.is_sweeping = True
        
        def sweep_motion():
            print(f"🔄 Начинаю качание от {start_angle}° до {end_angle}°")
            
            direction = 1  # 1 = вперед, -1 = назад
            current = start_angle
            
            while self.is_sweeping:
                # Движение к цели
                if direction == 1:
                    for angle in range(int(current), int(end_angle) + 1, 2):
                        if not self.is_sweeping:
                            break
                        self.move_to_angle(angle, smooth=False)
                        time.sleep(0.02 * speed)
                    current = end_angle
                    direction = -1
                else:
                    for angle in range(int(current), int(start_angle) - 1, -2):
                        if not self.is_sweeping:
                            break
                        self.move_to_angle(angle, smooth=False)
                        time.sleep(0.02 * speed)
                    current = start_angle
                    direction = 1
                
                if not continuous:
                    break
        
        self.sweep_thread = threading.Thread(target=sweep_motion)
        self.sweep_thread.daemon = True
        self.sweep_thread.start()
    
    def stop_sweep(self):
        """
        Остановка качания
        """
        if self.is_sweeping:
            print("🛑 Остановка качания")
            self.is_sweeping = False
            if self.sweep_thread:
                self.sweep_thread.join()
    
    def center(self):
        """
        Возврат в центральное положение (90°)
        """
        print("🎯 Возврат в центр")
        self.move_to_angle(90)
    
    def rotate_continuous(self, direction="clockwise", speed=2):
        """
        Имитация непрерывного вращения (для обычных серво)
        direction: "clockwise" или "counterclockwise"
        speed: скорость (1-5)
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
                    angle += 5
                    if angle > 180:
                        angle = 0
                else:
                    angle -= 5
                    if angle < 0:
                        angle = 180
                
                self.move_to_angle(angle, smooth=False)
                time.sleep(0.1 / speed)
        
        self.sweep_thread = threading.Thread(target=continuous_rotation)
        self.sweep_thread.daemon = True
        self.sweep_thread.start()
    
    def cleanup(self):
        """
        Очистка ресурсов
        """
        self.stop_sweep()
        self.pwm.stop()
        GPIO.cleanup()
        print("✅ Ресурсы очищены")

# Демонстрация работы серво мотора
if __name__ == "__main__":
    # Создаем контроллер серво
    servo = ServoController(servo_pin=15)
    
    try:
        print("\n=== ДЕМОНСТРАЦИЯ РАБОТЫ СЕРВО МОТОРА ===")
        
        # Тест 1: Основные позиции
        print("\n1️⃣  Тест основных позиций")
        positions = [0, 45, 90, 135, 180, 90]
        for pos in positions:
            servo.move_to_angle(pos)
            time.sleep(1)
        
        # Тест 2: Качание
        print("\n2️⃣  Тест качания (5 секунд)")
        servo.sweep(30, 150, speed=2, continuous=True)
        time.sleep(5)
        servo.stop_sweep()
        
        # Тест 3: Быстрое качание
        print("\n3️⃣  Быстрое качание (3 секунды)")
        servo.sweep(60, 120, speed=0.5, continuous=True)
        time.sleep(3)
        servo.stop_sweep()
        
        # Тест 4: Имитация непрерывного вращения
        print("\n4️⃣  Непрерывное вращение (5 секунд)")
        servo.rotate_continuous("clockwise", speed=3)
        time.sleep(5)
        servo.stop_sweep()
        
        # Возврат в центр
        servo.center()
        
        print("\n✅ Демонстрация завершена!")
        
    except KeyboardInterrupt:
        print("\n⚠️  Прерывание пользователем")
    
    finally:
        servo.cleanup()

# Простые функции для быстрого использования
def quick_move(angle, pin=15):
    """Быстрый поворот на угол"""
    servo = ServoController(pin)
    try:
        servo.move_to_angle(angle)
        time.sleep(2)
    finally:
        servo.cleanup()

def quick_sweep(duration=5, pin=15):
    """Быстрое качание"""
    servo = ServoController(pin)
    try:
        servo.sweep(0, 180, speed=2, continuous=True)
        time.sleep(duration)
    finally:
        servo.cleanup()

# Примеры использования:
# quick_move(45)  # Поворот на 45°
# quick_sweep(10)  # Качание 10 секунд