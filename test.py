#!/usr/bin/env python3
"""
Система управления мотором через PCA9685
Полная диагностика и тестирование
"""

import smbus
import time
import sys

class MotorController:
    def __init__(self):
        """Инициализация контроллера мотора"""
        self.bus = smbus.SMBus(1)
        self.PCA9685_ADDRESS = 0x40
        self.CHANNEL = 0
        self.PWM_FREQ = 50  # 50Hz для серво
        
        # Значения для MG996R серво (в тиках от 0-4095)
        # MG996R требует: 1ms-2ms импульсы при 50Hz
        # При 50Hz: период = 20ms, 4096 тиков = 20ms
        # 1ms = 205 тиков, 2ms = 410 тиков
        self.SERVO_MIN = 205   # 1ms импульс (0°)
        self.SERVO_MAX = 410   # 2ms импульс (180°)  
        self.SERVO_CENTER = 307 # 1.5ms импульс (90°)
        
        print("🔧 Инициализация системы управления мотором...")
        
    def scan_i2c_devices(self):
        """Сканирование I2C устройств"""
        print("\n📡 Сканирование I2C устройств...")
        devices = []
        for addr in range(0x03, 0x78):
            try:
                self.bus.read_byte(addr)
                devices.append(hex(addr))
                print(f"   ✅ Найдено устройство: {hex(addr)}")
            except:
                pass
        
        if hex(self.PCA9685_ADDRESS) in devices:
            print(f"   ✅ PCA9685 найден по адресу {hex(self.PCA9685_ADDRESS)}")
            return True
        else:
            print(f"   ❌ PCA9685 НЕ найден по адресу {hex(self.PCA9685_ADDRESS)}")
            return False
    
    def init_pca9685(self):
        """Инициализация PCA9685 драйвера"""
        print("\n⚙️  Инициализация PCA9685...")
        try:
            # Включение PCA9685
            self.bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, 0x00)
            time.sleep(0.1)
            
            # Установка частоты PWM (~50Hz для серво)
            # Формула: частота = 25MHz / (4096 * (prescale + 1))
            # Для 50Hz: prescale = 121
            prescale = 121
            
            # Переводим в режим сна для изменения частоты
            self.bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, 0x10)
            time.sleep(0.1)
            
            # Устанавливаем prescale
            self.bus.write_byte_data(self.PCA9685_ADDRESS, 0xFE, prescale)
            time.sleep(0.1)
            
            # Выходим из режима сна
            self.bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, 0x00)
            time.sleep(0.1)
            
            # Включаем автоувеличение адреса
            self.bus.write_byte_data(self.PCA9685_ADDRESS, 0x00, 0xA0)
            time.sleep(0.1)
            
            print("   ✅ PCA9685 инициализирован успешно")
            return True
            
        except Exception as e:
            print(f"   ❌ Ошибка инициализации PCA9685: {e}")
            return False
    
    def set_pwm(self, channel, value):
        """Установка PWM значения на канале"""
        try:
            # Вычисляем базовый адрес регистра для канала
            base_reg = 0x06 + 4 * channel
            
            # ON время = 0 (начинаем с начала цикла)
            self.bus.write_byte_data(self.PCA9685_ADDRESS, base_reg, 0)      # ON_L
            self.bus.write_byte_data(self.PCA9685_ADDRESS, base_reg + 1, 0)  # ON_H
            
            # OFF время = value (длительность импульса)
            self.bus.write_byte_data(self.PCA9685_ADDRESS, base_reg + 2, value & 0xFF)      # OFF_L
            self.bus.write_byte_data(self.PCA9685_ADDRESS, base_reg + 3, (value >> 8) & 0xFF)  # OFF_H
            
            return True
        except Exception as e:
            print(f"   ❌ Ошибка установки PWM: {e}")
            return False
    
    def test_basic_pwm(self):
        """Базовое тестирование PWM сигналов"""
        print("\n🔍 Базовое тестирование PWM...")
        
        test_values = [0, 150, 300, 450, 600, 0]
        
        for value in test_values:
            print(f"   📊 Устанавливаем PWM = {value}")
            if self.set_pwm(self.CHANNEL, value):
                print(f"   ✅ PWM {value} установлен успешно")
            else:
                print(f"   ❌ Ошибка установки PWM {value}")
            
            time.sleep(2)
            
            # Запрашиваем обратную связь от пользователя
            response = input(f"   ❓ Есть ли реакция мотора на PWM {value}? (y/n/stop): ").lower()
            if response == 'y':
                print(f"   ✅ Мотор реагирует на PWM {value}")
            elif response == 'stop':
                print("   ⏹️  Тестирование остановлено пользователем")
                break
            else:
                print(f"   ❌ Нет реакции на PWM {value}")
    
    def test_servo_range(self):
        """Тестирование диапазона серво мотора"""
        print("\n🎯 Тестирование диапазона серво мотора...")
        
        # Плавное движение от минимума к максимуму
        print("   📈 Плавное движение от минимума к максимуму...")
        for value in range(self.SERVO_MIN, self.SERVO_MAX + 1, 10):
            print(f"   📊 PWM = {value}")
            self.set_pwm(self.CHANNEL, value)
            time.sleep(0.1)
        
        time.sleep(1)
        
        # Обратно к центру
        print("   📍 Возврат в центральную позицию...")
        self.set_pwm(self.CHANNEL, self.SERVO_CENTER)
        time.sleep(1)
        
        # Запрашиваем обратную связь
        response = input("   ❓ Было ли движение мотора? (y/n): ").lower()
        return response == 'y'
    
    def test_dc_motor_mode(self):
        """Тестирование режима DC мотора"""
        print("\n🔄 Тестирование режима DC мотора...")
        print("   ℹ️  Если это DC мотор, попробуем различные скорости...")
        
        # Различные скорости для DC мотора
        speeds = [0, 1000, 2000, 3000, 4095, 0]
        
        for speed in speeds:
            print(f"   ⚡ Скорость = {speed} ({speed/4095*100:.1f}%)")
            self.set_pwm(self.CHANNEL, speed)
            time.sleep(2)
            
            response = input(f"   ❓ Есть ли вращение на скорости {speed}? (y/n/stop): ").lower()
            if response == 'y':
                print(f"   ✅ Мотор вращается на скорости {speed}")
                return True
            elif response == 'stop':
                break
        
        return False
    
    def test_all_channels(self):
        """Тестирование всех каналов драйвера"""
        print("\n🔌 Тестирование всех каналов драйвера...")
        
        test_value = 300
        
        for channel in range(16):
            print(f"   📡 Тестируем канал {channel}...")
            self.set_pwm(channel, test_value)
            time.sleep(1)
            
            response = input(f"   ❓ Есть ли реакция на канале {channel}? (y/n/skip): ").lower()
            if response == 'y':
                print(f"   ✅ Канал {channel} работает!")
                self.CHANNEL = channel
                return True
            elif response == 'skip':
                continue
            
            # Выключаем канал
            self.set_pwm(channel, 0)
        
        return False
    
    def interactive_control(self):
        """Интерактивное управление мотором"""
        print("\n🎮 Интерактивное управление мотором")
        print("   Команды:")
        print("   - Число (0-4095): установить PWM значение")
        print("   - 'min': минимальное значение серво")
        print("   - 'max': максимальное значение серво") 
        print("   - 'center': центральная позиция серво")
        print("   - 'sweep': плавное движение")
        print("   - 'stop': остановить мотор")
        print("   - 'quit': выход")
        
        while True:
            try:
                command = input(f"\n   Канал {self.CHANNEL} > ").strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'stop':
                    self.set_pwm(self.CHANNEL, 0)
                    print("   ⏹️  Мотор остановлен")
                elif command == 'min':
                    self.set_pwm(self.CHANNEL, self.SERVO_MIN)
                    print(f"   📍 Установлено минимальное значение: {self.SERVO_MIN}")
                elif command == 'max':
                    self.set_pwm(self.CHANNEL, self.SERVO_MAX)
                    print(f"   📍 Установлено максимальное значение: {self.SERVO_MAX}")
                elif command == 'center':
                    self.set_pwm(self.CHANNEL, self.SERVO_CENTER)
                    print(f"   📍 Установлена центральная позиция: {self.SERVO_CENTER}")
                elif command == 'sweep':
                    print("   🔄 Выполняем плавное движение...")
                    for i in range(5):
                        # К максимуму
                        for val in range(self.SERVO_MIN, self.SERVO_MAX, 5):
                            self.set_pwm(self.CHANNEL, val)
                            time.sleep(0.02)
                        # К минимуму
                        for val in range(self.SERVO_MAX, self.SERVO_MIN, -5):
                            self.set_pwm(self.CHANNEL, val)
                            time.sleep(0.02)
                    self.set_pwm(self.CHANNEL, self.SERVO_CENTER)
                elif command.isdigit():
                    value = int(command)
                    if 0 <= value <= 4095:
                        self.set_pwm(self.CHANNEL, value)
                        print(f"   📊 PWM установлен: {value}")
                    else:
                        print("   ❌ Значение должно быть от 0 до 4095")
                else:
                    print("   ❌ Неизвестная команда")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"   ❌ Ошибка: {e}")
    
    def run_diagnostics(self):
        """Запуск полной диагностики"""
        print("🚀 ЗАПУСК ДИАГНОСТИКИ СИСТЕМЫ УПРАВЛЕНИЯ МОТОРОМ")
        print("=" * 60)
        
        # Шаг 1: Сканирование I2C
        if not self.scan_i2c_devices():
            print("\n❌ КРИТИЧЕСКАЯ ОШИБКА: PCA9685 не найден!")
            print("   Проверьте подключение I2C проводов")
            return False
        
        # Шаг 2: Инициализация PCA9685
        if not self.init_pca9685():
            print("\n❌ КРИТИЧЕСКАЯ ОШИБКА: Не удалось инициализировать PCA9685!")
            return False
        
        # Шаг 3: Базовое тестирование PWM
        print(f"\n🎯 Тестируем канал {self.CHANNEL}...")
        self.test_basic_pwm()
        
        # Шаг 4: Определение типа мотора
        print(f"\n🤔 Определяем тип мотора...")
        
        motor_works = False
        
        # Тест как серво
        if self.test_servo_range():
            print("   ✅ Мотор работает как СЕРВО!")
            motor_works = True
        else:
            # Тест как DC мотор
            if self.test_dc_motor_mode():
                print("   ✅ Мотор работает как DC МОТОР!")
                motor_works = True
        
        # Шаг 5: Если мотор не работает
        if not motor_works:
            print("\n❓ Мотор не реагирует на команды. Возможные причины:")
            print("   1. 🔋 Недостаточное питание")
            print("   2. 🔌 Неправильное подключение мотора")
            print("   3. ⚡ Неисправность мотора")
            print("   4. 📡 Неисправность канала драйвера")
            
            # Предлагаем тест других каналов
            test_other = input("\n❓ Протестировать другие каналы? (y/n): ").lower()
            if test_other == 'y':
                if self.test_all_channels():
                    motor_works = True
        
        # Шаг 6: Интерактивное управление
        if motor_works:
            print(f"\n🎉 УСПЕХ! Мотор работает на канале {self.CHANNEL}")
            interactive = input("❓ Запустить интерактивное управление? (y/n): ").lower()
            if interactive == 'y':
                self.interactive_control()
        
        # Финальная остановка
        self.set_pwm(self.CHANNEL, 0)
        print("\n✅ Диагностика завершена. Мотор остановлен.")
        
        return motor_works

def main():
    """Главная функция"""
    try:
        controller = MotorController()
        controller.run_diagnostics()
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Программа остановлена пользователем")
    except Exception as e:
        print(f"\n❌ КРИТИЧЕСКАЯ ОШИБКА: {e}")
        print("   Проверьте подключение и права доступа к I2C")
    finally:
        print("\n👋 До свидания!")

if __name__ == "__main__":
    main()