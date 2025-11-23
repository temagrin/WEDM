from enum import Enum
from math import sqrt
import matplotlib.pyplot as plt


class StepperFSM:
    class State(Enum):
        Idle = 'Idle'
        Accel = 'Accel'
        Cruise = 'Cruise'
        Decel = 'Decel'
        Pause = 'Pause'
        Done = 'Done'

    def __init__(self, acceleration, pause_max_acceleration, min_speed):
        self.state = self.State.Idle
        self.last_update_time = 0
        # Основные параметры движения
        self.acceleration = acceleration  # базовое ускорение и торможение
        self.pause_max_acceleration = pause_max_acceleration  # ускорение при паузе (обычно больше базового)
        self.pause_acceleration = 0

        self.min_speed = min_speed
        self.max_speed = 0

        self.start_speed = 0
        self.end_speed = 0

        # Позиции и расстояния по осям
        self.position = 0
        self.current_speed = 0
        self.total_dist = 0

        # Временные метки для состояний
        self.state_time = 0
        self.accel_time = 0
        self.cruise_time = 0
        self.decel_time = 0

        self.pause_start_speed = 0

    def start_move(self, distance, max_speed, start_speed=0, end_speed=0):
        self.total_dist = distance
        self.max_speed = max_speed
        self.position = 0
        self.current_speed = start_speed
        self.start_speed = start_speed
        self.end_speed = end_speed
        self.state_time = 0
        self.pause_acceleration = 0
        self._calculate_profile(distance)
        self.state = self.State.Accel
        self.last_update_time = 0


    def pause(self, value):
        if value == 0:
            self.resume()
            return

        if self.state == self.State.Decel:
            self.pause_acceleration = (value / 255.0) * (
                        self.pause_max_acceleration - self.acceleration) + self.acceleration
        else:
            self.pause_acceleration = (value / 255.0) * self.pause_max_acceleration

        if self.state not in (self.State.Pause, self.State.Idle, self.State.Done):
            self.pause_start_speed = self.current_speed
            self.state = self.State.Pause
            self.state_time = 0

    def resume(self):
        if self.state == self.State.Pause:
            remaining_distance = self.total_dist - self.position
            speed_threshold = 1e-3
            start_speed_for_resume = self.current_speed if self.current_speed > speed_threshold else self.min_speed
            self.current_speed = start_speed_for_resume
            self.start_speed = start_speed_for_resume
            self._calculate_profile(remaining_distance)
            self.state = self.State.Accel
            self.pause_acceleration = 0
            self.state_time = 0

    def _calculate_profile(self, remaining_distance):
        v0 = max(self.start_speed, self.min_speed)
        vmin_end = max(self.end_speed, self.min_speed)
        vmax = max(self.max_speed, self.min_speed)
        a = self.acceleration
        s = remaining_distance

        accel_time = (vmax - v0) / a if vmax > v0 else 0
        accel_dist = ((vmax + v0) / 2) * accel_time
        decel_dist = (vmax**2 - vmin_end**2) / (2 * a)

        if accel_dist + decel_dist > s:
            max_reachable_speed = sqrt(a * s + (v0**2 + vmin_end**2) / 2)
            max_reachable_speed = max(max_reachable_speed, self.min_speed)
            self.max_speed = min(max_reachable_speed, vmax)
            self.accel_time = (self.max_speed - v0) / a if self.max_speed > v0 else 0
            self.cruise_time = 0
            self.decel_time = (self.max_speed - vmin_end) / a
        else:
            self.max_speed = vmax
            self.accel_time = (self.max_speed - v0) / a
            self.decel_time = (self.max_speed - vmin_end) / a
            cruise_dist = s - accel_dist - decel_dist
            self.cruise_time = cruise_dist / self.max_speed if self.max_speed != 0 else 0

        self.state_time = 0

    def tick(self, now:int):
        if self.last_update_time==0:
            self.last_update_time = now
            return 0
        self.update((now - self.last_update_time)*0.000001)
        return self.current_speed

    def update(self, dt):
        if self.state in (self.State.Idle, self.State.Done):
            return

        self.state_time += dt

        if self.state == self.State.Pause:
            if self.current_speed > 0 and self.pause_acceleration > 0:
                prev_speed = self.current_speed
                self.current_speed = max(self.current_speed - self.pause_acceleration * dt, 0)
                avg_speed = (prev_speed + self.current_speed) / 2
                self.position += avg_speed * dt
            return

        if self.state == self.State.Accel:
            self.current_speed += self.acceleration * dt
            self.current_speed = min(max(self.current_speed, self.min_speed), self.max_speed)
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
            self.current_speed = max(self.current_speed, self.min_speed)
            self.position += self.current_speed * dt
            if self.state_time >= self.cruise_time:
                self.state_time = 0
                self.state = self.State.Decel
            if self.position >= self.total_dist:
                self.position = self.total_dist
                self.state = self.State.Done

        elif self.state == self.State.Decel:
            prev_speed = self.current_speed

            effective_accel = -max(self.acceleration,
                                   self.pause_acceleration if self.pause_acceleration > 0 else self.acceleration)

            if self.current_speed > self.min_speed:
                self.current_speed += effective_accel * dt
                if self.current_speed < self.min_speed:
                    self.current_speed = self.min_speed
            else:
                self.current_speed = self.min_speed

            avg_speed = (prev_speed + self.current_speed) / 2
            self.position += avg_speed * dt

            if self.position >= self.total_dist and abs(self.current_speed - self.min_speed) < 1e-6:
                self.state = self.State.Done

    def is_done(self):
        return self.state == self.State.Done


def run_test_case(description, pause_times, distance=18000, max_speed=2000):
    print(f"\n--- Test: {description} ---")
    fsm = StepperFSM(1000, 2000, 10)
    fsm.start_move(distance, max_speed, 100, 200)

    dt = 0.001
    time = 0.0
    pause_index = 0

    times, positions, speeds = [], [], []

    while not fsm.is_done():
        fsm.update(dt)

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

        positions.append(fsm.position)
        speeds.append(fsm.current_speed)

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

    plt.title(description)
    plt.show()


if __name__ == "__main__":
    # 1. Без прерываний
    run_test_case("No interruptions", [])

    # 2. Прерывание на ускорении (короткое)
    run_test_case("Pause during acceleration (short)",
                  [(0.5, "pause", 255), (0.6, "resume", 255)])

    run_test_case("Pause during acceleration (long)",
                  [(0.5, "pause", 255), (2.5, "resume", 255)])

    # 3. Прерывание на круизе
    run_test_case("Pause during cruise (short)",
                  [(2.5, "pause", 255), (2.8, "resume", 255)])
    run_test_case("Pause part during cruise (short)",
                  [(2.5, "pause", 60), (2.8, "resume", 255)])

    run_test_case("Pause during cruise (long)",
                  [(2.5, "pause", 255), (4.5, "resume", 255)])
    run_test_case("Pause part during cruise (long)",
                  [(2.5, "pause", 60), (4.5, "resume", 255)])

    run_test_case("Pause during decel (long)",
                  [(9.5, "pause", 255), (10.5, "resume", 255)])
    run_test_case("Pause part during decel (long)",
                  [(9.5, "pause", 128), (10.5, "resume", 255)])

    run_test_case("Pause during decel (short)",
                  [(9.5, "pause", 255), (9.8, "resume", 255)])
    run_test_case("Pause part during decel (short)",
                  [(9.5, "pause", 128), (9.8, "resume", 255)])
