from dataclasses import dataclass
from decimal import Decimal, getcontext
from typing import List, Tuple
import scurvebeta as scb
import numpy as np

getcontext().prec = 40
D = Decimal

@dataclass
class SpeedSegment:
    steps: int
    speed_steps_per_sec: float

def calculate_target_segment_size(v: float, min_size=30, max_size=60) -> int:
    if v <= 50:
        return min_size
    elif v >= 1000:
        return max_size
    else:
        return int(30 + 30 * (v - 50) / 950)

def plan_path_with_scurve(
        paths: List[Tuple[int, int]],
        typical_speed: int,
        typical_accel: int,
        max_packs_per_segment: int = 60
) -> List[List[SpeedSegment]]:

    segs = []
    last = (D(0), D(0))
    for x, y in paths:
        p = (D(x), D(y))
        dx, dy = p[0] - last[0], p[1] - last[1]
        length = float((dx**2 + dy**2).sqrt())
        if length > 0:
            segs.append({'dx': dx, 'dy': dy, 'length_steps': length})
        last = p

    all_segments = []
    v_max_float = float(typical_speed)
    a_max_float = float(typical_accel)

    for seg_idx, seg in enumerate(segs):
        L = seg['length_steps']

        # *** РЕАЛИСТИЧНАЯ v_peak ***
        v_peak = min(v_max_float, (2 * a_max_float * 0.7 * L)**0.5)
        motion_time = scb.motionTime(v_peak, a_max_float, L)

        print(f"Сегмент {seg_idx}: L={L:.0f}, v_peak={v_peak:.0f}")

        # *** РАВНОМЕРНО ПО ДЛИНЕ, точно max_packs_per_segment точек ***
        n_samples = max_packs_per_segment * 4  # 4 точки на пачку
        t = np.linspace(0, motion_time, n_samples)
        positions = scb.sCurve(t, motion_time, 0, L)

        # *** СТРОГО max_packs_per_segment ПАЧЕК ***
        seg_segments = []
        total_target_steps = L

        for pack_idx in range(max_packs_per_segment):
            # *** РАВНОМЕРНОЕ РАЗДЕЛЕНИЕ по дистанции ***
            s_start = (pack_idx / max_packs_per_segment) * L
            s_end = ((pack_idx + 1) / max_packs_per_segment) * L

            # *** НАХОДИМ среднюю скорость на участке ***
            t_start = np.interp(s_start, positions, t)
            t_end = np.interp(s_end, positions, t)

            # *** Ищем ближайшие точки ***
            i_start = np.argmin(np.abs(positions - s_start))
            i_end = np.argmin(np.abs(positions - s_end))

            v_avg = 0.0
            if i_end > i_start:
                avg_pos = np.mean(positions[i_start:i_end])
                avg_t = np.mean(t[i_start:i_end])
                # Аналитическая скорость: Δs/Δt участка
                v_avg = (s_end - s_start) / (t_end - t_start)

            steps = int((s_end - s_start) + 0.5)
            v_clamped = min(max(v_avg, 0.1), v_max_float)  # 0.1 минимум

            if steps >= 10:  # минимум 10 шагов
                seg_segments.append(SpeedSegment(
                    steps=steps,
                    speed_steps_per_sec=v_clamped
                ))

        all_segments.append(seg_segments)

        speeds = [s.speed_steps_per_sec for s in seg_segments]
        total_steps = sum(s.steps for s in seg_segments)
        print(f"  ✅ {len(seg_segments)} пачек ({total_steps}/{L:.0f} шагов), "
              f"v: {min(speeds):.0f}...{max(speeds):.0f}")

    return all_segments

if __name__ == '__main__':
    paths = [(10000, 20000), (800, 30000), (1800, 2000)]
    typical_speed = 100000  # 51200
    typical_accel = 100 * 128  # 25600
    for p in plan_path_with_scurve(paths, typical_speed, typical_accel, max_packs_per_segment=60):
        print("--------")
        for s in p:
            print(s.steps, s.speed_steps_per_sec)

    # profiles = plan_path_with_scurve(paths, typical_speed, typical_accel, max_packs_per_segment=60)
    #
    # print("\n=== 🎯 РЕЗУЛЬТАТ ===")
    # total_steps = 0
    # for seg_idx, seg_segs in enumerate(profiles):
    #     seg_steps = sum(s.steps for s in seg_segs)
    #     speeds = [s.speed_steps_per_sec for s in seg_segs]
    #     print(f"\nСегмент {seg_idx}: {len(seg_segs)} пачек = {seg_steps} шагов")
    #     print("  " + " | ".join(f"{s.steps:3d}@{s.speed_steps_per_sec:6.0f}"
    #                             for s in seg_segs[::len(seg_segs)//6 or 1][:6]))
    #     total_steps += seg_steps
    #
    # print(f"\n✅ ИТОГО: {total_steps} шагов в {sum(map(len, profiles))} пачек!")
