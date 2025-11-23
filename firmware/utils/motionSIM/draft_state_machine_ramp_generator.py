import math
import matplotlib.pyplot as plt
from math import gcd, ceil, sqrt

class StepperFSM:
    class State:
        Idle = 'Idle'
        Accel = 'Accel'
        Cruise = 'Cruise'
        Decel = 'Decel'
        Pause = 'Pause'
        Done = 'Done'

    def __init__(self, acceleration, pause_max_acceleration, min_speed):
        self.acceleration = acceleration      # базовое ускорение (и торможение)
        self.pause_max_acceleration = pause_max_acceleration # ускорение торможения при вызове pause? заведом больше acceleration
        self.pause_acceleration = 0
        self.min_speed = min_speed
        self.max_speed = 0                    
        self.distance_x = 0
        self.distance_y = 0    
        self.current_speed = 0                
        self.position = 0
        self.position_x = 0
        self.position_y = 0     
        self.state = self.State.Idle
        self.state_time = 0
        self.accel_time = 0
        self.cruise_time = 0
        self.decel_time = 0
        self.pause_start_speed = 0
        self.total_dist = 0
        self.ratio_x = 0
        self.ratio_y = 0
        

    def start_move(self, distance_x, distance_y, max_speed):
        self.distance_x = distance_x
        self.distance_y = distance_y
        self.total_dist = math.sqrt(distance_x**2 + distance_y**2)
        self.ratio_x = abs(distance_x)/self.total_dist
        self.ratio_y = abs(distance_y)/self.total_dist
        
        self.max_speed = max_speed
        self.position = 0
        self.current_speed = 0
        self.state_time = 0
        self._calculate_profile()
        self.state = self.State.Accel

    def pause(self, value):
        if value == 0:
            self.resume()
            return
        # Штатное ускорение по умолчанию
        self.pause_acceleration = (value / 255.0) * (self.pause_max_acceleration - self.acceleration) + self.acceleration
        # self.pause_acceleration теперь всегда >= self.acceleration
        if self.state not in [self.State.Pause, self.State.Idle, self.State.Done]:
            self.pause_start_speed = self.current_speed
            self.state = self.State.Pause
            self.state_time = 0
            
    def resume(self):
        if self.state == self.State.Pause:
            remaining_distance = self.total_dist - self.position
            speed_threshold = 1e-3  # порог скорости близкой к 0
            if self.current_speed < speed_threshold and remaining_distance > 0:
                # Переходим к догоняющему торможению — стартуем с 0 скорости заново (Decel/Accel)
                self.current_speed = 0
                self._recalc_profile_after_resume(0, remaining_distance, self.max_speed)
                self.state = self.State.Accel
            else:
                self._recalc_profile_after_resume(self.current_speed, remaining_distance, self.max_speed)
                self.state = self.State.Accel
            self.pause_acceleration = 0
            self.state_time = 0

    def _integer_ratios(self):
        scale_factor = 1000000  # или больше для точности
        ix = int(round(self.ratio_x * scale_factor))
        iy = int(round(self.ratio_y * scale_factor))
        if ix == 0 or iy == 0:
            return ix, iy
        g = gcd(ix, iy)
        return ix // g, iy // g

    def _integer_ratios(self):
        scale_factor = 1000000  # для точности
        ix = int(round(self.ratio_x * scale_factor))
        iy = int(round(self.ratio_y * scale_factor))
        if ix == 0 or iy == 0:
            return ix, iy
        g = gcd(ix, iy)
        return ix // g, iy // g

    def _calculate_profile(self):
        vmin = self.min_speed
        a = self.acceleration

        vmax = self.max_speed if self.max_speed > vmin else vmin

        # Расчет идеального профиля
        accel_dist = ((vmax ** 2) - (vmin ** 2)) / (2.0 * a)
        if 2 * accel_dist >= self.total_dist:
            max_reachable_speed = sqrt(a * self.total_dist + vmin ** 2)
            vmax = max_reachable_speed if max_reachable_speed > vmin else vmin

        # Получаем целочисленные пропорции для осей
        ratio_int_x, ratio_int_y = self._integer_ratios()

        # Идеальные интервалы в микросекундах
        interval_x = 1e6 / (vmax * self.ratio_x) if self.ratio_x > 0 else 0
        interval_y = 1e6 / (vmax * self.ratio_y) if self.ratio_y > 0 else 0

        # Минимальный базовый целочисленный интервал для обоих направлений
        candidates = []
        if ratio_int_x != 0 and interval_y > 0:
            candidates.append(ceil(interval_y / ratio_int_x))
        if ratio_int_y != 0 and interval_x > 0:
            candidates.append(ceil(interval_x / ratio_int_y))
        base_interval = max(candidates) if candidates else 1
        if base_interval < 1:
            base_interval = 1

        # Ограничение максимального базового интервала, чтобы не остановить движение
        max_allowed_base_interval = 1e6 / vmin if vmin > 0 else 1e6
        if base_interval > max_allowed_base_interval:
            base_interval = max_allowed_base_interval

        # Итоговые интервалы с учётом пропорций
        interval_x_us = base_interval * ratio_int_y
        interval_y_us = base_interval * ratio_int_x

        # Итоговые скорости по осям из целочисленных интервалов
        vmax_x = 1e6 / interval_x_us if interval_x_us != 0 else 0
        vmax_y = 1e6 / interval_y_us if interval_y_us != 0 else 0

        vmax_adjusted = min(vmax_x / self.ratio_x if self.ratio_x > 0 else float('inf'),
                           vmax_y / self.ratio_y if self.ratio_y > 0 else float('inf'))

        # Гарантируем минимум чуть выше минимальной скорости
        if vmax_adjusted < vmin * 1.05:
            vmax_adjusted = vmin * 1.05

        self.max_speed = vmax_adjusted

        # Перерасчёт профиля скорости с новыми скоростями
        accel_dist = ((self.max_speed ** 2) - (vmin ** 2)) / (2.0 * a)
        if 2 * accel_dist >= self.total_dist:
            self.accel_time = max(0.001, (self.max_speed - vmin) / a)
            self.cruise_time = 0
            self.decel_time = self.accel_time
        else:
            self.accel_time = max(0.001, (self.max_speed - vmin) / a)
            self.decel_time = self.accel_time
            cruise_dist = self.total_dist - 2 * accel_dist
            self.cruise_time = max(0, cruise_dist / self.max_speed if self.max_speed != 0 else 0)

        self.state_time = 0

    def _recalc_profile_after_resume(self, start_speed, remaining_distance, max_speed):
        vmin = self.min_speed
        a = self.acceleration
        v0 = start_speed if start_speed > vmin else vmin
        vmax = max_speed if max_speed > vmin else vmin
        s = remaining_distance
        ratio_int_x, ratio_int_y = self._integer_ratios()
        accel_time = (vmax - v0) / a if vmax > v0 else 0
        accel_dist = ((vmax + v0) / 2) * accel_time
        decel_dist = (vmax**2 - vmin**2) / (2 * a)

        interval_x_f = 1e6 / (vmax * self.ratio_x) if self.ratio_x > 0 else 0
        interval_y_f = 1e6 / (vmax * self.ratio_y) if self.ratio_y > 0 else 0

        candidates = []
        if ratio_int_x != 0 and interval_y_f > 0:
            candidates.append(ceil(interval_y_f / ratio_int_x))
        if ratio_int_y != 0 and interval_x_f > 0:
            candidates.append(ceil(interval_x_f / ratio_int_y))
        base_interval = max(candidates) if candidates else 1
        if base_interval < 1:
            base_interval = 1

        max_allowed_base_interval = 1e6 / vmin if vmin > 0 else 1e6
        if base_interval > max_allowed_base_interval:
            base_interval = max_allowed_base_interval

        interval_x_us = base_interval * ratio_int_y
        interval_y_us = base_interval * ratio_int_x

        speed_x = 1e6 / interval_x_us if interval_x_us != 0 else 0
        speed_y = 1e6 / interval_y_us if interval_y_us != 0 else 0

        vmax_adjusted = min(speed_x / self.ratio_x if self.ratio_x > 0 else float('inf'),
                           speed_y / self.ratio_y if self.ratio_y > 0 else float('inf'))

        if accel_dist + decel_dist > s:
            max_reachable_speed = sqrt(a * s + (v0**2)/2)
            if max_reachable_speed < vmin:
                max_reachable_speed = vmin
            self.max_speed = max_reachable_speed if max_reachable_speed < vmax_adjusted else vmax_adjusted
            self.accel_time = max(0.001, (self.max_speed - v0) / a if self.max_speed > v0 else 0)
            self.cruise_time = 0
            self.decel_time = max(0.001, (self.max_speed - vmin) / a)
        else:
            self.max_speed = vmax_adjusted if vmax_adjusted > vmin else vmin
            self.accel_time = max(0.001, (self.max_speed - v0) / a)
            self.decel_time = max(0.001, (self.max_speed - vmin) / a)
            cruise_dist = s - accel_dist - decel_dist
            self.cruise_time = max(0, cruise_dist / self.max_speed if self.max_speed != 0 else 0)
        self.state_time = 0
        
    def update(self, dt):
        if self.state in [self.State.Idle, self.State.Done]:
            return
        self.state_time += dt
        if self.state == self.State.Pause:
            if self.current_speed > 0 and self.pause_acceleration > 0:
                prev_speed = self.current_speed
                self.current_speed -= self.pause_acceleration * dt
                if self.current_speed < 0:
                    self.current_speed = 0
                avg_speed = (prev_speed + self.current_speed) / 2.0
                self.position += avg_speed * dt
            return

        if self.state == self.State.Accel:
            self.current_speed += self.acceleration * dt
            if self.current_speed > self.max_speed:
                self.current_speed = self.max_speed
            if self.current_speed < self.min_speed:
                self.current_speed = self.min_speed
            self.position += self.current_speed * dt
            if self.state_time >= self.accel_time:
                self.state_time = 0
                if self.cruise_time > 0:
                    self.state = self.State.Cruise
                else:
                    self.state = self.State.Decel
            if self.position >= self.total_dist:
                self.position = self.total_dist
                self.state = self.State.Done

        elif self.state == self.State.Cruise:
            if self.current_speed < self.min_speed:
                self.current_speed = self.min_speed
            self.position += self.current_speed * dt
            if self.state_time >= self.cruise_time:
                self.state_time = 0
                self.state = self.State.Decel
            if self.position >= self.total_dist:
                self.position = self.total_dist
                self.state = self.State.Done

        elif self.state == self.State.Decel:
            prev_speed = self.current_speed

            # Плавное приближение к min_speed
            effective_accel = -max(self.acceleration, self.pause_acceleration if self.pause_acceleration > 0 else self.acceleration)

            # Если мы можем не снижать скорость ниже min_speed:
            if self.current_speed > self.min_speed:
                self.current_speed += effective_accel * dt
                if self.current_speed < self.min_speed:
                    self.current_speed = self.min_speed
            else:
                # Если скорость уже на минимуме, удерживаем ее
                self.current_speed = self.min_speed

            avg_speed = (prev_speed + self.current_speed) / 2.0
            self.position += avg_speed * dt

            # Завершить, если достигли позиции и скорость равна min_speed
            if self.position >= self.total_dist and abs(self.current_speed - self.min_speed) < 1e-6:
                self.position = self.total_dist
                self.state = self.State.Done

    def get_intervals(self, dt):
        current_speed_x = self.current_speed * self.ratio_x
        current_speed_y = self.current_speed * self.ratio_y
        current_interval_x_us = 0
        if current_speed_x!=0:
            current_interval_x_us = 1e6 / current_speed_x
        
        current_interval_y_us = 0
        if current_speed_x!=0:
            current_interval_y_us = 1e6 / current_speed_y
        return current_interval_x_us, current_interval_y_us # если равен нулю - значит не шагаем


    def set_min_speed(self, min_speed):
        self.min_speed = min_speed
    
    def set_accelerations(self, acceleration, pause_max_acceleration):
        self.acceleration = acceleration
        self.pause_max_acceleration = pause_max_acceleration
        

    def is_done(self):
        return self.state == self.State.Done

    def __str__(self):
        return (f"State: {self.state}, Pos: {self.position:.2f}, Speed: {self.current_speed:.2f}, "
                f"AccelT: {self.accel_time:.2f}, CruiseT: {self.cruise_time:.2f}, DecelT: {self.decel_time:.2f}")


def run_test_case(description, pause_times, acceleration=1000, distance_x=15000, distance_y=11000, max_speed=2000):
    print(f"\n--- Test: {description} ---")
    fsm = StepperFSM(acceleration, acceleration*2, 10)
    fsm.start_move(distance_x, distance_y, max_speed)

    dt = 0.001
    time = 0.0
    pause_index = 0

    times, positions,positions_x,positions_y, speeds = [], [], [], [], []

    while not fsm.is_done():
        fsm.update(dt)
        intervals_x_us, intervals_y_us = fsm.get_intervals(dt) 
        # fsm.emulate_steper_set_intervals(dt, intervals_x_us, intervals_y_us) # эмуляция коррекции изменения скорости шаговых двигателей, которые не могут шагать не кратно 1 микросекундк.

        time += dt
        # Check if we need to pause/resume
        if pause_index < len(pause_times):
            event_time, action, parg = pause_times[pause_index]
            if abs(time - event_time) < dt / 2:
                if action == "pause":
                    fsm.pause(parg)
                elif action == "resume":
                    fsm.resume()
                pause_index += 1

        times.append(time)

        
        speeds.append(fsm.get_current_speed())

    #задача подбирать интервалы таким образом чтобы колиечество сделанных шагов равнялось количеству шагов в задание
    print(f"Ожидаем сделать по оси X {distance_x}, сделали виртуально {fsm.get_current_position_x()}")
    print(f"Ожидаем сделать по оси Y {distance_y}, сделали виртуально {fsm.get_current_position_y()}")
    
    # Plot
    fig, ax1 = plt.subplots()
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (steps)', color='tab:blue')
    ax1.plot(times, positions, color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax2 = ax1.twinx()
    ax2.set_ylabel('Speed (steps/s)', color='tab:red')
    ax2.plot(times, speeds, color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')
    ax3 = ax2.twinx()
    
    ax3.set_ylabel('Position X (steps)', color='tab:green')
    ax3.plot(times,  positions_x, color='tab:green')
    ax3.tick_params(axis='y', labelcolor='tab:green')

    ax4 = ax2.twinx()
    ax4.set_ylabel('Position Y (steps)', color='tab:cyan')
    ax4.plot(times, positions_y, color='tab:cyan')
    ax4.tick_params(axis='y', labelcolor='tab:cyan')
    
    plt.title(description)
    plt.show()


if __name__ == "__main__":
    # 1. Без прерываний
    run_test_case("No interruptions", [])

    # 2. Прерывание на ускорении (короткое)
    run_test_case("Pause during acceleration (short)",
                  [(0.5, "pause", 255), (0.6, "resume", 255)])

    # 3. Прерывание на круизе
    run_test_case("Pause during cruise",
                  [(1.5, "pause", 255), (2.5, "resume", 255)])

    # 4. Прерывание на торможении
    run_test_case("Pause during deceleration",
                  [(3.3, "pause", 255), (4.0, "resume", 255)])

    # 5. Два прерывания, разной длительности
    run_test_case("Two pauses, different lengths",
                  [(0.5, "pause", 255), (1.0, "resume", 255), (2.0, "pause", 255), (3.0, "resume", 255)])
    # 6. Прерывание на круизе
    run_test_case("Pause part during cruise",
                  [(1.5, "pause", 40), (2.5, "resume", 255)])
    # 7. Прерывание на круизе
    run_test_case("Pause part during cruise_resume as pause(0)",
                  [(1.5, "pause", 40), (2.5, "pause", 0)])
    
    # 4. Прерывание на торможении
    run_test_case("Pause part during deceleration",
                  [(3.3, "pause", 40), (4.0, "resume", 255)])
