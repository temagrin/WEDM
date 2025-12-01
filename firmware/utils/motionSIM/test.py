import unittest

from calc import calc_intervals, SCALE

def ticks_from_parts(int_part, frac_part):
    return int_part * SCALE + frac_part

def total_time(steps, int_part, frac_part):
    return steps * ticks_from_parts(int_part, frac_part)

class TestMotionIntervals(unittest.TestCase):
    def test_high_speed(self):
        ix, fx, iy, fy = calc_intervals(500, 500, 10000)
        ticks_x = ticks_from_parts(ix, fx)
        # Реалистично: ~70 мкс при 10000 шагов/сек по траектории
        self.assertLess(ticks_x, 100)

    def test_ratios_sync(self):
        ix, fx, iy, fy = calc_intervals(1, 100, 200)
        T_x = total_time(1, ix, fx)
        T_y = total_time(100, iy, fy)
        # Допуск 1% от общего времени приемлемо для синхронизации
        self.assertLess(abs(T_x - T_y), max(T_x, T_y) * 0.01)

if __name__ == '__main__':
    unittest.main(verbosity=2)