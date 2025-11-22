class MotorState:
    def __init__(self):
        self.stepLastTime = 0
        self.stepInterval = 0
        self.errorIncrement = 0
        self.errorAccumulator = 0
        self.direction = True

SCALE = 1_000_000_000

def setSpeed(state: MotorState, speed: int):
    if speed == 0:
        state.stepInterval = 0
        return
    state.direction = speed > 0
    step_interval = 1_000_000 / abs(speed)  # мкс на шаг
    int_part = int(step_interval)
    frac_part = step_interval - int_part
    state.stepInterval = int_part
    state.errorIncrement = int(frac_part * SCALE)

def needStep(state: MotorState, now: int):
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
    return True



def test_fixed_point_with_speed_output(expected_speed = 130000, max_steps = 10000, delta_t = 1):
    state = MotorState()
    setSpeed(state, expected_speed)

    now = 0
    steps = 0
    times = []

    while steps < max_steps:
        if needStep(state, now):
            times.append(now)
            steps += 1
        now += delta_t

    total_time = times[-1] - times[0] if len(times) > 1 else 1
    actual_speed = (len(times) - 1) / (total_time / 1_000_000)

    theoretical_steps = total_time / (1_000_000 / expected_speed)

    step_difference = len(times) - theoretical_steps
    relative_diff_percent = (step_difference / theoretical_steps) * 100 if theoretical_steps != 0 else 0

    print(f"Expected speed (stepInterval based): {expected_speed} steps/s")
    print(f"Actual speed (measured): {actual_speed:.2f} steps/s")
    print(f"Speed difference: {actual_speed - expected_speed:.2f} steps/s")
    print(f"Relative speed difference: {(actual_speed - expected_speed) / expected_speed * 100:.2f}%")
    print(f"Theoretical steps: {theoretical_steps:.2f}")
    print(f"Actual steps: {len(times)}")
    print(f"Step count difference: {step_difference:.2f}")
    print(f"Relative step difference: {relative_diff_percent:.2f}%")


if __name__ == '__main__':
    test_fixed_point_with_speed_output(expected_speed = 10, max_steps = 100, delta_t = 1)
