import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial.tools.list_ports
import threading
import time
import queue
import re

# --- Настройки Serial ---
BAUDRATE = 115200
ARDUINO_PORT = None # Будет найден автоматически
SERIAL_TIMEOUT = 1 # секунда

# --- Найдём порт Arduino ---
def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Arduino" in p.description or "CH340" in p.description or "CP210" in p.description or "USB" in p.description:
            print(f"Найден Arduino на порту: {p.device}")
            return p.device
    return None

ARDUINO_PORT = find_arduino_port()
if not ARDUINO_PORT:
    print("Arduino не найден!")
    exit(1)

# --- Переменные для GUI ---
# PID коэффициенты по умолчанию (увеличенные для работы с малыми скоростями и большими ШИМ)
default_values = {
    'left': {'p': 100.0, 'i': 0.0, 'd': 0.0, 'kff': 1.0}, # Примерно 100 для P
    'right': {'p': 100.0, 'i': 0.0, 'd': 0.0, 'kff': 1.0},
    'velocities': {'linear': 0.0, 'angular': 0.0}
}

# Очередь для данных Serial
data_queue = queue.Queue()

# Переменные для хранения данных графиков
# Используем deque для эффективного добавления/удаления старых данных
from collections import deque
MAX_DATA_POINTS = 100 # Максимум точек на графике

time_buffer = deque(maxlen=MAX_DATA_POINTS)
left_target_buffer = deque(maxlen=MAX_DATA_POINTS)
left_current_buffer = deque(maxlen=MAX_DATA_POINTS)
right_target_buffer = deque(maxlen=MAX_DATA_POINTS)
right_current_buffer = deque(maxlen=MAX_DATA_POINTS)

# --- ПЕРЕМЕННЫЕ ДЛЯ PID И ЦЕЛЕВЫХ СКОРОСТЕЙ ---
# Для упрощения, будем рассчитывать требуемый ШИМ на стороне Python
# и отправлять его как команду motor_id, pwm_value
target_linear_vel = 0.0
target_angular_vel = 0.0
target_left_wheel_speed = 0.0
target_right_wheel_speed = 0.0

# PID коэффициенты (хранятся как переменные для доступа из обновления)
Kp_l = default_values['left']['p']
Ki_l = default_values['left']['i']
Kd_l = default_values['left']['d']
kff_l = default_values['left']['kff']

Kp_r = default_values['right']['p']
Ki_r = default_values['right']['i']
Kd_r = default_values['right']['d']
kff_r = default_values['right']['kff']

# Буферы для ошибок PID (для простоты используем список)
error_history_left = {'p': 0, 'i': 0, 'd': 0, 'prev_time': time.time()}
error_history_right = {'p': 0, 'i': 0, 'd': 0, 'prev_time': time.time()}

# Параметры робота
WHEEL_BASE_PYTHON = 0.105 # м # Используем значение из Arduino-скетча
# SPEED_TO_PWM_FACTOR_PYTHON не используется напрямую в PID, только для feedforward если нужно
MAX_LINEAR_SPEED_MPS_PYTHON = 0.3 # Примерное значение
MAX_PWM_OUTPUT = 255 # Максимальный ШИМ

# --- Функция для отправки команд в Serial ---
def send_command(cmd_str):
    if ser and ser.is_open:
        full_cmd = cmd_str + "\n"
        ser.write(full_cmd.encode('utf-8'))
        # print(f"Отправлено: {full_cmd.strip()}") # Для отладки

# --- Функция для вычисления ШИМ по PID ---
def calculate_pwm_output(error, last_error, integral, derivative, dt, kp, ki, kd, kff, max_pwm=MAX_PWM_OUTPUT):
    # P (Пропорциональная)
    p_term = kp * error
    
    # I (Интегральная)
    # Учет времени важен для I. dt - это время между измерениями.
    # integral хранит сумму (error * dt)
    new_integral = integral + error * dt
    # Анти-обмотка интеграла: ограничиваем интегральную сумму так, чтобы I-составляющая не превышала max_pwm
    # max_integral = max_pwm / ki если ki != 0, иначе max_pwm
    max_integral = max_pwm / abs(ki) if ki != 0 else max_pwm
    new_integral = max(min(new_integral, max_integral), -max_integral)
    i_term = ki * new_integral
    
    # D (Дифференциальная)
    # derivative уже передаётся как (error - last_error) / dt
    d_term = kd * derivative
    
    output = p_term + i_term + d_term
    
    # Feedforward (простая компенсация мертвой зоны)
    # kff_l/r теперь просто добавляется к положительному или отрицательному ШИМ
    # Применяем kff к общему выходу
    sign_output = 1 if output > 0 else (-1 if output < 0 else 0)
    output += sign_output * kff
    
    # Ограничение ШИМ
    output = max(min(output, max_pwm), -max_pwm)
    
    # Умножаем на kff как коэффициент компенсации трения
    output *= kff
    
    return output, new_integral, derivative # Возвращаем также обновленную производную

# --- Функция для обновления значений на Arduino ---
def update_arduino_settings():
    global target_linear_vel, target_angular_vel
    global Kp_l, Ki_l, Kd_l, kff_l
    global Kp_r, Ki_r, Kd_r, kff_r
    global error_history_left, error_history_right

    # Обновляем параметры из GUI
    target_linear_vel = scale_linear.get()
    target_angular_vel = scale_angular.get()
    
    Kp_l = scale_left_p.get()
    Ki_l = scale_left_i.get()
    Kd_l = scale_left_d.get()
    kff_l = scale_left_kff.get()
    
    Kp_r = scale_right_p.get()
    Ki_r = scale_right_i.get()
    Kd_r = scale_right_d.get()
    kff_r = scale_right_kff.get()

    # Вычисляем целевые скорости колёс
    target_left_wheel_speed = target_linear_vel - (target_angular_vel * WHEEL_BASE_PYTHON / 2.0)
    target_right_wheel_speed = target_linear_vel + (target_angular_vel * WHEEL_BASE_PYTHON / 2.0)

    # Получаем последние текущие скорости из буфера (если есть)
    current_left_speed = left_current_buffer[-1] if left_current_buffer else 0.0
    current_right_speed = right_current_buffer[-1] if right_current_buffer else 0.0

    # Вычисляем ошибки
    error_left = target_left_wheel_speed - current_left_speed
    error_right = target_right_wheel_speed - current_right_speed

    # Вычисляем dt для PID (время с последнего обновления)
    current_time = time.time()
    dt_left = current_time - error_history_left['prev_time']
    dt_right = current_time - error_history_right['prev_time']
    # Обновляем время
    error_history_left['prev_time'] = current_time
    error_history_right['prev_time'] = current_time

    # Вычисляем ШИМ с помощью PID
    pwm_left, new_int_left, new_deriv_left = calculate_pwm_output(
        error_left, error_history_left['p'], error_history_left['i'], error_history_left['d'], dt_left,
        Kp_l, Ki_l, Kd_l, kff_l
    )
    pwm_right, new_int_right, new_deriv_right = calculate_pwm_output(
        error_right, error_history_right['p'], error_history_right['i'], error_history_right['d'], dt_right,
        Kp_r, Ki_r, Kd_r, kff_r
    )

    # Обновляем историю ошибок
    error_history_left = {'p': error_left, 'i': new_int_left, 'd': new_deriv_left, 'prev_time': current_time}
    error_history_right = {'p': error_right, 'i': new_int_right, 'd': new_deriv_right, 'prev_time': current_time}

    # Отправляем ШИМ-команды на Arduino
    send_command(f"0,{int(pwm_left)}")
    send_command(f"1,{int(pwm_right)}")

    # Также обновляем буферы целевой скорости для графика (для отладки)
    # Используем последнее время из time_buffer или текущее, если буфер пуст
    plot_time = time_buffer[-1] if time_buffer else current_time
    left_target_buffer.append(target_left_wheel_speed)
    right_target_buffer.append(target_right_wheel_speed)


# --- Функция для обработки данных Serial в отдельном потоке ---
def serial_reader():
    global ser
    ser = None
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=SERIAL_TIMEOUT)
        print(f"Подключено к {ARDUINO_PORT}")
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                # print(f"Получено: {line}") # Для отладки
                if line:
                    data_queue.put(line)
            time.sleep(0.01) # Небольшая задержка
    except serial.SerialException as e:
        print(f"Ошибка Serial: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Соединение Serial закрыто.")


# --- Переменная для хранения ID таймера update_plots ---
update_timer_id = None

# --- Функция для обновления графиков ---
def update_plots():
    try: # --- Добавляем try ---
        global time_buffer, left_target_buffer, left_current_buffer, right_target_buffer, right_current_buffer
        global update_timer_id # Объявляем глобальной

        # Обработать все данные из очереди
        while not data_queue.empty():
            try:
                line = data_queue.get_nowait()
                # Парсим строку с текущими скоростями
                # Ожидаем формат: "currentLeftWheelSpeed,currentRightWheelSpeed"
                # match = re.match(r'^([+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?)\s*,\s*([+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?)$', line)
                parts = line.split(',')
                if len(parts) == 2:
                     try:
                         current_left = float(parts[0])
                         current_right = float(parts[1])
                         
                         # Вычисляем целевые скорости на основе линейной и угловой (для отображения на графике)
                         # Используем текущие значения из GUI или переменных
                         v_linear = target_linear_vel # scale_linear.get() # Используем глобальную переменную
                         v_angular = target_angular_vel # scale_angular.get() # Используем глобальную переменную
                         
                         target_left_calc = v_linear - (v_angular * WHEEL_BASE_PYTHON / 2.0)
                         target_right_calc = v_linear + (v_angular * WHEEL_BASE_PYTHON / 2.0)

                         # Добавляем в буферы
                         current_time = time.time()
                         time_buffer.append(current_time)
                         # left_target_buffer.append(target_left_calc) # Заменено на обновление в update_arduino_settings
                         # right_target_buffer.append(target_right_calc) # Заменено на обновление в update_arduino_settings
                         left_current_buffer.append(current_left)
                         right_current_buffer.append(current_right)
                     except ValueError:
                         # Если не получилось распарсить числа
                         pass
            except queue.Empty:
                pass # Ничего не делаем, если очередь пуста

        # Обновляем буферы целевой скорости для графика (теперь всегда обновляется, даже если не было новых данных от Serial)
        # Это нужно, чтобы график целевой скорости был актуален
        if time_buffer: # Проверяем, что буфер времени не пуст
            current_time = time_buffer[-1] # Берём последнее время
            v_linear = target_linear_vel
            v_angular = target_angular_vel
            target_left_calc = v_linear - (v_angular * WHEEL_BASE_PYTHON / 2.0)
            target_right_calc = v_linear + (v_angular * WHEEL_BASE_PYTHON / 2.0)
            # Обновляем только последнее значение в буфере целевой скорости, если время совпадает
            # Или добавляем новое, если буферы целевой скорости короче
            if len(left_target_buffer) < len(time_buffer):
                 # Если буфер целевой скорости короче, добавляем новое значение
                 left_target_buffer.append(target_left_calc)
                 right_target_buffer.append(target_right_calc)
            elif len(left_target_buffer) == len(time_buffer):
                 # Если длины совпадают, обновляем последнее значение
                 left_target_buffer[-1] = target_left_calc
                 right_target_buffer[-1] = target_right_calc
            # else: # Если буфер целевой скорости длиннее - не должно происходить при maxlen


        # Обновляем графики
        ax1.clear()
        ax1.set_ylim(-0.5, 0.5)  # Фиксированный диапазон скорости
        plotted_left = False # Флаг для левой оси
        if len(time_buffer) > 0:
            # Убедимся, что длины совпадают перед отрисовкой
            # Берём минимальную длину из времени и целевой/текущей скорости
            min_len = min(len(time_buffer), len(left_target_buffer), len(left_current_buffer))
            if min_len > 0:
                t_data = list(time_buffer)[-min_len:]
                target_data = list(left_target_buffer)[-min_len:]
                current_data = list(left_current_buffer)[-min_len:]

                ax1.plot(t_data, target_data, label='Left Target (m/s)', color='blue')
                ax1.plot(t_data, current_data, label='Left Current (m/s)', color='red', linestyle='--')
                plotted_left = True # Устанавливаем флаг, если графики нарисованы
        ax1.set_title('Left Wheel Speed')
        ax1.set_ylabel('Speed (m/s)')
        if plotted_left: # Вызываем legend только если были нарисованы линии
            ax1.legend(loc='upper left')
        ax1.grid(True)

        ax2.clear()
        ax2.set_ylim(-0.5, 0.5)  # Фиксированный диапазон скорости
        plotted_right = False # Флаг для правой оси
        if len(time_buffer) > 0:
            # Убедимся, что длины совпадают перед отрисовкой
            min_len = min(len(time_buffer), len(right_target_buffer), len(right_current_buffer))
            if min_len > 0:
                t_data = list(time_buffer)[-min_len:]
                target_data = list(right_target_buffer)[-min_len:]
                current_data = list(right_current_buffer)[-min_len:]

                ax2.plot(t_data, target_data, label='Right Target (m/s)', color='green')
                ax2.plot(t_data, current_data, label='Right Current (m/s)', color='orange', linestyle='--')
                plotted_right = True # Устанавливаем флаг, если графики нарисованы
        ax2.set_title('Right Wheel Speed')
        ax2.set_ylabel('Speed (m/s)')
        ax2.set_xlabel('Time (s)')
        if plotted_right: # Вызываем legend только если были нарисованы линии
            ax2.legend(loc='upper left')
        ax2.grid(True)

        canvas.draw()

        # Вызываем update_arduino_settings для расчёта и отправки ШИМ
        update_arduino_settings()

        # Запускаем следующий цикл без отмены текущего уже выполненного таймера
        update_timer_id = root.after(50, update_plots) # Повтор через 50 мс (частота обновления PID)
    except tk.TclError: # --- Перехватываем TclError ---
        # print("Ошибка Tkinter (возможно, связанная с after или Canvas).") # Для отладки
        pass # Игнорируем ошибку и НЕ запускаем update_plots снова
    # except Exception as e: # Необязательно: ловить другие исключения
    #     print(f"Неожиданная ошибка в update_plots: {e}")
    #     pass # Или снова pass, или действия по вашему усмотрению


# --- Создание GUI ---
root = tk.Tk()
root.title("PID Tuning Tool (Simple Protocol) - Adjusted for Speed->PWM")

# Создаём фреймы
frame_sliders = tk.Frame(root)
frame_sliders.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)

frame_plot = tk.Frame(root)
frame_plot.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=10, pady=10)

# --- Ползунки ---
# Левый мотор
ttk.Label(frame_sliders, text="LEFT MOTOR").grid(row=0, column=0, columnspan=2, sticky="w")
scale_left_p = tk.Scale(frame_sliders, from_=0, to=2000, resolution=1.0, orient=tk.HORIZONTAL, label="P (0-2000)", length=200)
scale_left_p.set(default_values['left']['p'])
scale_left_p.grid(row=1, column=0, padx=5, pady=5)

scale_left_i = tk.Scale(frame_sliders, from_=0, to=500, resolution=0.1, orient=tk.HORIZONTAL, label="I (0-500)", length=200)
scale_left_i.set(default_values['left']['i'])
scale_left_i.grid(row=1, column=1, padx=5, pady=5)

scale_left_d = tk.Scale(frame_sliders, from_=0, to=500, resolution=0.1, orient=tk.HORIZONTAL, label="D (0-500)", length=200)
scale_left_d.set(default_values['left']['d'])
scale_left_d.grid(row=2, column=0, padx=5, pady=5)

scale_left_kff = tk.Scale(frame_sliders, from_=0.0, to=2.0, resolution=0.01, orient=tk.HORIZONTAL, label="kff (0.0-2.0)", length=200)
scale_left_kff.set(default_values['left']['kff'])
scale_left_kff.grid(row=2, column=1, padx=5, pady=5)

# Правый мотор
ttk.Label(frame_sliders, text="RIGHT MOTOR").grid(row=0, column=2, columnspan=2, sticky="w")
scale_right_p = tk.Scale(frame_sliders, from_=0, to=2000, resolution=1.0, orient=tk.HORIZONTAL, label="P (0-2000)", length=200)
scale_right_p.set(default_values['right']['p'])
scale_right_p.grid(row=1, column=2, padx=5, pady=5)

scale_right_i = tk.Scale(frame_sliders, from_=0, to=500, resolution=0.1, orient=tk.HORIZONTAL, label="I (0-500)", length=200)
scale_right_i.set(default_values['right']['i'])
scale_right_i.grid(row=1, column=3, padx=5, pady=5)

scale_right_d = tk.Scale(frame_sliders, from_=0, to=500, resolution=0.1, orient=tk.HORIZONTAL, label="D (0-500)", length=200)
scale_right_d.set(default_values['right']['d'])
scale_right_d.grid(row=2, column=2, padx=5, pady=5)

scale_right_kff = tk.Scale(frame_sliders, from_=0.0, to=2.0, resolution=0.01, orient=tk.HORIZONTAL, label="kff (0.0-2.0)", length=200)
scale_right_kff.set(default_values['right']['kff'])
scale_right_kff.grid(row=2, column=3, padx=5, pady=5)

# Целевые скорости
ttk.Label(frame_sliders, text="TARGET VELOCITIES").grid(row=3, column=0, columnspan=2, sticky="w")
scale_linear = tk.Scale(frame_sliders, from_=-0.2, to=0.2, resolution=0.01, orient=tk.HORIZONTAL, label="Linear (m/s)", length=200)
scale_linear.set(default_values['velocities']['linear'])
scale_linear.grid(row=4, column=0, padx=5, pady=5)

scale_angular = tk.Scale(frame_sliders, from_=-1.57, to=1.57, resolution=0.01, orient=tk.HORIZONTAL, label="Angular (rad/s)", length=200)
scale_angular.set(default_values['velocities']['angular'])
scale_angular.grid(row=4, column=1, padx=5, pady=5)

# Кнопки обнуления скоростей
btn_reset_linear = ttk.Button(frame_sliders, text="Reset Linear", command=lambda: scale_linear.set(0.0))
btn_reset_linear.grid(row=5, column=0, padx=5, pady=5)

btn_reset_angular = ttk.Button(frame_sliders, text="Reset Angular", command=lambda: scale_angular.set(0.0))
btn_reset_angular.grid(row=5, column=1, padx=5, pady=5)

# Кнопка для отправки (теперь не нужна, так как отправка идёт в update_plots)
# btn_update = ttk.Button(frame_sliders, text="Update Arduino", command=update_arduino_settings)
# btn_update.grid(row=5, column=0, columnspan=4, pady=10)
ttk.Label(frame_sliders, text="Adjust sliders to tune PID. P/I/D should be much larger than before.").grid(row=6, column=0, columnspan=4, pady=5)

# --- Графики Matplotlib ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
canvas = FigureCanvasTkAgg(fig, master=frame_plot)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Запуск потока Serial
serial_thread = threading.Thread(target=serial_reader, daemon=True)
serial_thread.start()

# Запуск обновления графиков (и PID)
update_timer_id = root.after(100, update_plots) # Сохраняем ID первого таймера

# --- Функция для очистки при закрытии окна ---
def on_closing():
    global update_timer_id
    if update_timer_id:
        root.after_cancel(update_timer_id)
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing) # Привязываем функцию к событию закрытия

# Запуск GUI
root.mainloop()
