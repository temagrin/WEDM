import math
from math import sqrt


SCALE = 1_000_000_000

class MotorState2:
    def __init__(self):
        self.stepLastTime = 0
        self.stepInterval = 0
        self.errorIncrement = 0
        self.errorAccumulator = 0
        self.direction = True
        self.remaining_steps = 0


class MotorController2:
    def __init__(self):
        self.xmotor = MotorState2()
        self.ymotor = MotorState2()
        self.current_x_position = 0
        self.current_y_position = 0

        self.finish_speed2 = 0 # квадрат конечной скорости мастер мотора

        self.decel_flag = False

        self.ratio_x = 0.0
        self.ratio_y = 0.0
        self.total_steps = 0

        self.current_speed = 0
        self.max_speed = 0
        self.min_axes_speed = 0
        self.start_speed = 0
        self.finish_speed = 0 # квадрат конечной скорости
        self.acceleration_factor = 1 # ускорение шаг/сек^2  умножить на 0.0001 сек ( вызов обновления скорости каждые 100 микросекунд)
        self.deceleration_factor = 1


    @staticmethod
    def ajustSpeed(state: MotorState2, speed):
        if speed == 0:
            state.stepInterval = 0 # остановка мотора
            return
        step_interval = 1_000_000 / abs(speed)  # мкс на шаг
        int_part = int(step_interval)
        frac_part = step_interval - int_part
        state.stepInterval = int_part
        state.errorIncrement = int(frac_part * SCALE)

    def setSpeed(self, state: MotorState2, speed):
        """ режим постоянного вращения """
        if speed == 0:
            state.stepInterval = 0 # остановка мотора
            return
        state.remaining_steps = -1
        state.direction = speed > 0
        self.ajustSpeed(state, speed)


    @staticmethod
    def needStep(state: MotorState2, now: int):
        """ проверка нужно ли текущему мотору совершить шаг """
        if state.stepInterval == 0:
            return False
        if (now - state.stepLastTime) < state.stepInterval:
            return False
        state.errorAccumulator += state.errorIncrement
        if state.errorAccumulator > SCALE:
            # сдвигаем следующий шаг на 1 мкс позже
            state.stepLastTime = now + 1
            state.errorAccumulator -= SCALE
        else:
            state.stepLastTime = now
        if state.remaining_steps != 0:
            if state.remaining_steps > 0:
                state.remaining_steps -= 1
            return True
        return False

    def need_decel(self):
        if self.decel_flag:
            return True
        total = self.calc_remaining_total()
        left = total * 2 * self.deceleration_factor
        right = (self.current_speed**2 - self.finish_speed2) * 0.0001  # delta_t фиксированная
        self.decel_flag = left < right
        return self.decel_flag


    def fsm_tick(self):
        # пора тормозить? до self.finish_speed
        if self.need_decel():
            self.current_speed -= self.deceleration_factor
            if self.current_speed < self.finish_speed:
                self.current_speed = self.finish_speed
        # иначе ускоряемся до self.max_speed
        elif self.current_speed<=self.max_speed:
            self.current_speed += self.acceleration_factor
        self.update_xy_speed()


    def update_xy_speed(self):
        x_motor_speed = self.current_speed * self.ratio_x
        y_motor_speed = self.current_speed * self.ratio_y
        self.ajustSpeed(self.xmotor, x_motor_speed)
        self.ajustSpeed(self.ymotor, y_motor_speed)


    def tick(self, now: int):
        has_move = False
        if self.needStep(self.xmotor, now):
            has_move = True
            self.total_steps -=1
            if self.xmotor.direction:
                self.current_x_position += 1
            else:
                self.current_x_position -= 1
        if self.needStep(self.ymotor, now):
            self.total_steps -=1
            has_move = True
            if self.ymotor.direction:
                self.current_y_position += 1
            else:
                self.current_y_position -= 1
        return has_move

    def ready(self):
        return self.xmotor.remaining_steps==0 and self.ymotor.remaining_steps==0

    def calc_remaining_total(self):
        return sqrt(((self.xmotor.remaining_steps)**2) + ((self.ymotor.remaining_steps)**2)) + 50


    def look_ahead(self, target_x:int, target_y:int, next_target_x:int, next_target_y:int):
        dx_a = target_x - self.current_x_position
        dy_a = target_y - self.current_y_position
        dx_b = next_target_x - target_x
        dy_b = next_target_y - target_y

        if (dx_a != 0 and (dx_a > 0) != (dx_b > 0)) or (dy_a != 0 and (dy_a > 0) != (dy_b > 0)):
            return 0.0
        alpha = math.atan2(dy_a, dx_a)
        beta = math.atan2(dy_b, dx_b)

        diff = abs(beta - alpha)
        if diff > math.pi:
            diff = 2 * math.pi - diff
        if diff >= math.pi / 2:
            return 0.0
        k = math.cos(diff)
        return k


    def move_to(self, target_x:int, target_y:int, max_speed: int, junction_speed: int):
        d_x = target_x - self.current_x_position
        d_y = target_y - self.current_y_position

        if d_x == 0 and d_y == 0:
            return False # никуда не надо перемещаться
        if max_speed<=0:
            return False # с нулевой или отрицательной скоростью тоже не поедем
        print()
        print()
        print(f"Текущая позиция на момент поступления задачи {self.current_x_position} {self.current_y_position}")
        print(f"Поступила задача двигаться к точкам {target_x} {target_y} со скоростью {max_speed}")
        print(f"Расчитанное перемещение {d_x} {d_y}")

        if d_x>0:
            self.xmotor.direction = True
            self.xmotor.remaining_steps = d_x
        else:
            self.xmotor.direction = False
            self.xmotor.remaining_steps = int(d_x * -1)

        if d_y>0:
            self.ymotor.direction = True
            self.ymotor.remaining_steps = d_y
        else:
            self.ymotor.direction = False
            self.ymotor.remaining_steps = int(d_y * -1)

        print(f"Оставшиеся шаги x: {self.xmotor.remaining_steps}")
        print(f"Оставшиеся шаги y: {self.ymotor.remaining_steps}")

        total_dist = sqrt((d_x*d_x) + (d_y*d_y))

        self.ratio_x = self.xmotor.remaining_steps/total_dist
        self.ratio_y = self.ymotor.remaining_steps/total_dist

        # стартовая, и максимальная по мастер оси
        self.current_speed = self.finish_speed  # берем прошлую junction_speed
        self.max_speed = max_speed # берем нынешнюю max_speed
        self.finish_speed = junction_speed # ставим текщуюу конечную скорость
        self.finish_speed2 = junction_speed**2
        self.total_steps=self.xmotor.remaining_steps+self.ymotor.remaining_steps
        self.update_xy_speed()
        self.decel_flag = False
        return True # задача успешно поставлена
