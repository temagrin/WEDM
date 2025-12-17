from math import sqrt


SCALE = 1_000_000_000

class MotorState:
    def __init__(self):
        self.stepLastTime = 0
        self.stepInterval = 0
        self.errorIncrement = 0
        self.errorAccumulator = 0
        self.direction = True
        self.remaining_steps = 0


class MotorController:
    def __init__(self):
        self.xmotor = MotorState()
        self.ymotor = MotorState()
        self.current_x_position = 0
        self.current_y_position = 0
        self.ratio_x = 0
        self.ratio_y = 0
        self.current_total_dist = 0
        self.current_speed = 0
        self.max_speed = 0
        self.finish_speed = 0 # квадрат конечной скорости
        self.finish_speed2 = 0 # квадрат конечной скорости
        self.acceleration_factor =  2 # ускорение шаг/сек^2  умножить на 0.0001 сек ( вызов обновления скорости каждые 100 микросекунд)
        self.decceleration_factor = 12

    @staticmethod
    def ajustSpeed(state: MotorState, speed):
        """ режим постоянного вращения """
        if speed == 0:
            state.stepInterval = 0 # остановка мотора
            return
        step_interval = 1_000_000 / abs(speed)  # мкс на шаг
        int_part = int(step_interval)
        frac_part = step_interval - int_part
        state.stepInterval = int_part
        state.errorIncrement = int(frac_part * SCALE)

    def setSpeed(self, state: MotorState, speed):
        """ режим постоянного вращения """
        if speed == 0:
            state.stepInterval = 0 # остановка мотора
            return
        state.remaining_steps = -1
        state.direction = speed > 0
        self.ajustSpeed(state, speed)


    @staticmethod
    def needStep(state: MotorState, now: int):
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
        # если оставшиеся шаги не считаем(тоесть движение постоянное то state.remaining_steps=-1
        # в противном случае проверяем есть ли положительные невыполенные шаги
        if state.remaining_steps != 0:
            if state.remaining_steps > 0:
                state.remaining_steps -= 1
            return True
        return False

    def fsm_tick(self):
        elasted_dist = sqrt((self.xmotor.remaining_steps*self.xmotor.remaining_steps)
                            + (self.ymotor.remaining_steps*self.ymotor.remaining_steps))

        left = elasted_dist * 2 * self.acceleration_factor
        right = (self.current_speed**2 - self.finish_speed2) * 0.0001  # delta_t фиксированная
        if left <= right:
            self.current_speed -= self.acceleration_factor
            if self.current_speed < self.finish_speed:
                self.current_speed = self.finish_speed

        elif self.current_speed<=self.max_speed:
            self.current_speed += self.acceleration_factor

        x_motor_speed = self.current_speed * self.ratio_x
        y_motor_speed = self.current_speed * self.ratio_y
        # print(f"{now} {x_motor_speed}  {y_motor_speed}")
        self.ajustSpeed(self.xmotor, x_motor_speed)
        self.ajustSpeed(self.ymotor, y_motor_speed)

    def tick(self, now: int):
        has_move = False
        if self.needStep(self.xmotor, now):
            has_move = True
            if self.xmotor.direction:
                self.current_x_position += 1
            else:
                self.current_x_position -= 1
        if self.needStep(self.ymotor, now):
            has_move = True
            if self.ymotor.direction:
                self.current_y_position += 1
            else:
                self.current_y_position -= 1
        return has_move

    def ready(self):
        return self.xmotor.remaining_steps==0 and self.ymotor.remaining_steps==0

    def move_to(self, target_x:int, target_y:int, max_speed: int):
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
        self.current_total_dist = total_dist
        self.ratio_x = self.xmotor.remaining_steps/total_dist
        self.ratio_y = self.ymotor.remaining_steps/total_dist
        self.max_speed = max_speed
        self.finish_speed = 0
        self.finish_speed2 = 0


        return True # задача успешно поставлена
