#!/usr/bin/env python3
"""
Тестирование серво мотора через PCA9685
Подключение: Raspberry Pi 5 + PCA9685 + Servo
"""

import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


def initialize_servo():
    """Инициализация PCA9685 и серво мотора"""
    try:
        print("Инициализация I2C...")
        i2c = busio.I2C(board.SCL, board.SDA)

        print("Подключение к PCA9685...")
        pca = PCA9685(i2c)
        pca.frequency = 50  # 50 Гц для серво моторов

        print("Создание объекта серво на канале 0...")
        servo_motor = servo.Servo(pca.channels[0])

        print("✅ Инициализация успешна!")
        return pca, servo_motor

    except Exception as e:
        print(f"❌ Ошибка инициализации: {e}")
        print("\nПроверьте:")
        print("1. Включен ли I2C: sudo raspi-config")
        print("2. Подключены ли провода SDA/SCL")
        print("3. Установлены ли библиотеки")
        return None, None


def set_angle(servo_motor, angle):
    """Установить угол серво мотора"""
    try:
        if 0 <= angle <= 180:
            servo_motor.angle = angle
            print(f"✅ Серво повернуто на {angle}°")
            return True
        else:
            print("❌ Угол должен быть от 0 до 180 градусов")
            return False
    except Exception as e:
        print(f"❌ Ошибка поворота: {e}")
        return False


def smooth_move(servo_motor, start_angle, end_angle, steps=20, delay=0.05):
    """Плавное движение серво"""
    print(f"Плавное движение от {start_angle}° до {end_angle}°...")
    step_size = (end_angle - start_angle) / steps
    current_angle = start_angle

    for i in range(steps + 1):
        angle = int(current_angle)
        servo_motor.angle = angle
        print(f"  → {angle}°", end='\r')
        current_angle += step_size
        time.sleep(delay)
    print(f"  → {end_angle}° ✅")


def test_servo(servo_motor):
    """Тест основных позиций серво"""
    print("\n🔧 Тестирование серво мотора...")

    positions = [
        (90, "Центр"),
        (0, "Крайнее левое"),
        (180, "Крайнее правое"),
        (90, "Возврат в центр")
    ]

    for angle, description in positions:
        print(f"  {description} ({angle}°)...")
        set_angle(servo_motor, angle)
        time.sleep(1.5)

    print("✅ Тест завершен!")


def sweep_demo(servo_motor, cycles=3):
    """Демонстрация качания серво"""
    print(f"\n🔄 Качание серво ({cycles} циклов)...")

    for cycle in range(cycles):
        print(f"  Цикл {cycle + 1}/{cycles}")
        smooth_move(servo_motor, 0, 180, steps=30, delay=0.03)
        time.sleep(0.2)
        smooth_move(servo_motor, 180, 0, steps=30, delay=0.03)
        time.sleep(0.2)

    # Возврат в центр
    smooth_move(servo_motor, 0, 90, steps=15)
    print("✅ Качание завершено!")


def print_help():
    """Показать справку по командам"""
    print("\n📋 Доступные команды:")
    print("  0-180    - Поворот на указанный угол (например: 90)")
    print("  sweep    - Автоматическое качание туда-сюда")
    print("  test     - Тест основных позиций")
    print("  help     - Показать эту справку")
    print("  q        - Выход")


def main():
    """Основная функция программы"""
    print("🚀 Тестирование серво мотора с PCA9685")
    print("=" * 50)

    # Инициализация
    pca, servo_motor = initialize_servo()
    if not pca or not servo_motor:
        return

    print("\n🎯 Установка начальной позиции (90°)...")
    set_angle(servo_motor, 90)
    time.sleep(1)

    print_help()

    try:
        while True:
            print("\n" + "-" * 30)
            command = input("Введите команду: ").strip().lower()

            if command == 'q' or command == 'quit':
                break

            elif command == 'help' or command == 'h':
                print_help()

            elif command == 'sweep':
                sweep_demo(servo_motor)

            elif command == 'test':
                test_servo(servo_motor)

            elif command.isdigit():
                angle = int(command)
                set_angle(servo_motor, angle)

            elif command == '':
                continue

            else:
                print(f"❌ Неизвестная команда: '{command}'")
                print("Введите 'help' для списка команд")

    except KeyboardInterrupt:
        print("\n⚠️  Программа прервана пользователем")

    finally:
        print("\n🔄 Завершение работы...")
        print("Возврат серво в центральное положение...")
        set_angle(servo_motor, 90)
        time.sleep(0.5)

        print("Отключение PCA9685...")
        pca.deinit()
        print("✅ Программа завершена!")


if __name__ == "__main__":
    main()