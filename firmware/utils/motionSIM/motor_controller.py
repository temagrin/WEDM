from utils.motionSIM.calc import SCALE_FACTOR_32_BITS


class MotorSpeedState:
    def __init__(self):
        self.stepLastTime = 0
        self.stepInterval = 0
        self.errorIncrement = 0
        self.errorAccumulator = 0

class MotorBresenhamState:
    def __init__(self, targetX, targetY, dirX, dirY, finished=False):
        self.targetX=targetX
        self.targetY=targetY
        self.taskCurrentX=0
        self.taskCurrentY=0
        self.errorPosition=targetX-targetY
        self.finished=finished
        self.dirX = dirX
        self.dirY = dirY


class MotorController:
    def __init__(self):
        self.speed_state = MotorSpeedState()
        self.buf = []
        self.active_state = MotorBresenhamState(0,0, True, True, True)
        self.current_x_position = 0
        self.current_y_position = 0

    def ready(self):
        return not self.buf and self.active_state.finished

    def addToBuffer(self, dirX, dirY, stepsX, stepsY, intSpeedPart, errorIncrement):
        self.buf.append((dirX, dirY, stepsX, stepsY, intSpeedPart, errorIncrement))

    def popXY(self):
        if not self.active_state.finished:
            return
        if self.buf:
            dirX, dirY, stepsX, stepsY, intSpeedPart, errorIncrement = self.buf.pop()
            self.active_state = MotorBresenhamState(stepsX, stepsY, dirX, dirY)
            self.speed_state.stepInterval = intSpeedPart
            self.speed_state.errorIncrement = errorIncrement
            self.speed_state.errorAccumulator = 0


    @staticmethod
    def needStep(state: MotorSpeedState, now: int):
        """ проверка нужно ли текущему мотору совершить шаг """
        if state.stepInterval == 0:
            return False
        if (now - state.stepLastTime) < state.stepInterval:
            return False
        state.errorAccumulator += state.errorIncrement
        if state.errorAccumulator > SCALE_FACTOR_32_BITS:
            # сдвигаем следующий шаг на 1 мкс позже
            state.stepLastTime = now + 1
            state.errorAccumulator -= SCALE_FACTOR_32_BITS
        else:
            state.stepLastTime = now
        return True

    @staticmethod
    def need_bresenham_step(state: MotorBresenhamState):
        need_x = False
        need_y = False
        if state.finished:
            return need_x, need_y
        if state.taskCurrentX == state.targetX and state.taskCurrentY == state.targetY:
            state.finished = True
            return need_x, need_y

        error2 = state.errorPosition * 2
        if error2 > -state.targetY:
            state.errorPosition -= state.targetY
            state.taskCurrentX += 1
            need_x = True
        if error2 < state.targetX:
            state.errorPosition += state.targetX
            state.taskCurrentY += 1
            need_y = True
        return need_x, need_y

    def tick(self, now: int):
        has_move = False
        if self.needStep(self.speed_state, now):
            has_move = True
            step_x, step_y = self.need_bresenham_step(self.active_state)
            if step_x:
                self.current_x_position += 1 if self.active_state.dirX else -1
            if step_y:
                self.current_y_position += 1 if self.active_state.dirY else -1

        self.popXY()
        return has_move

