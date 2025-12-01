from decimal import Decimal, getcontext, ROUND_FLOOR, ROUND_HALF_UP

getcontext().prec = 40

SCALE_FACTOR_32_BITS = Decimal(0x100000000)
US_PER_SEC = Decimal(1_000_000)

def calculate_dda_params(dx:int, dy:int, target_speed_steps_per_sec:int):
    dx_dec = Decimal(dx)
    dy_dec = Decimal(dy)
    speed_dec = Decimal(target_speed_steps_per_sec)

    distance_steps_sq = dx_dec**2 + dy_dec**2
    distance_steps = distance_steps_sq.sqrt()

    if distance_steps == 0:
        return None

    total_time_sec = distance_steps / speed_dec
    total_time_us = total_time_sec * US_PER_SEC

    t_total_us_rounded = max(
        total_time_us.to_integral_value(rounding=ROUND_HALF_UP),
        Decimal(1)
    )

    def axis_params(axis_steps):
        if axis_steps == 0:
            return 0, 0
        time_per_step = t_total_us_rounded / axis_steps
        step_int = int(time_per_step.to_integral_value(rounding=ROUND_FLOOR))
        frac_part = time_per_step - Decimal(step_int)
        error_inc = min(int(frac_part * SCALE_FACTOR_32_BITS + Decimal('0.5')), 0xFFFFFFFF)
        return min(step_int, 0xFFFFFFFF), error_inc

    sx, ex = axis_params(dx_dec)
    sy, ey = axis_params(dy_dec)

    check = abs(Decimal(sx) + Decimal(ex)/SCALE_FACTOR_32_BITS) * dx_dec - \
                abs(Decimal(sy) + Decimal(ey)/SCALE_FACTOR_32_BITS) * dy_dec
    print(f"Ratio error: {check:.1e} мкс") if abs(check) > 1e-6 else None
    return sx, ex, sy, ey

if __name__ == '__main__':
    dx_val = 128*200*4
    dy_val = 128*200*4
    speed_sps = 128*200*2 # 2об/сек шагового мотора 200 шагов на оборот, микрошаг драйвера 1/128 без интерполяции LV8729

    stepIntervalX, errorIncrementX, stepIntervalY, errorIncrementY  = calculate_dda_params(dx_val, dy_val, speed_sps)
    print(f"stepIntervalX: {stepIntervalX}, errorIncrementX: {errorIncrementX}")
    print(f"stepIntervalY: {stepIntervalY}, errorIncrementY: {errorIncrementY}")
