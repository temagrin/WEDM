import matplotlib.pyplot as plt
import numpy as np


def adaptive_speed_calculator(t_points, x_points):
    v_points = np.zeros_like(x_points, dtype=float)
    last_change_time = t_points[0]
    last_pos = x_points[0]
    last_speed = 0.0
    last_change_idx = 0

    for i in range(0, len(x_points) - 1):
        if x_points[i] != last_pos:
            dt = t_points[i] - last_change_time
            if dt > 0:
                last_speed = (x_points[i] - last_pos) / dt
            else:
                last_speed = 0
            last_pos = x_points[i]
            last_change_time = t_points[i]
            last_change_idx = i

        v_points[i] = last_speed

    if last_change_idx < len(x_points) - 1:
        v_points[last_change_idx + 1:] = 0

    return v_points


def plot_points(title, logs, x_points, y_points, t_points):
    N = 20
    # добавим точек в конце чтобы зафиксировать остановку последнего движения
    for i in range(N):
        t_points.append(t_points[-1] + 1)  # время у нас дискретно и равномерно, поэтому так можно - для начала и конца
        t_points.append(t_points[-1] + 1)
        x_points.append(x_points[-1])
        y_points.append(y_points[-1])

    origin_t_points = t_points
    x_points = np.array(x_points)
    y_points = np.array(y_points)
    t_points = np.array(t_points)

    prepend_x = np.zeros(N)
    prepend_y = np.zeros(N)

    x_points = np.concatenate([prepend_x, x_points])
    y_points = np.concatenate([prepend_y, y_points])

    v_x_adaptive = adaptive_speed_calculator(t_points, x_points)
    v_y_adaptive = adaptive_speed_calculator(t_points, y_points)

    v_x_filtered = v_x_adaptive
    v_y_filtered = v_y_adaptive


    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(24, 12))

    # XY в пространстве
    ax1.plot(x_points, y_points, marker='o', linestyle='-', markersize=2)
    ax1.set_aspect('equal', adjustable='datalim')  # одинаковый масштаб по осям X и Y
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.grid(True)


    # XY от времени T
    ax2.plot(origin_t_points, x_points, label='Движение по X', marker='o', markersize=2, color='y')
    ax2.plot(origin_t_points, y_points, label='Движение по Y', marker='x', markersize=2, color='g')
    ax2.set_xlabel('Время')
    ax2.set_ylabel('Позиция')
    ax2.legend(loc='center left', bbox_to_anchor=(1.05, 0.9))
    ax2.grid(True)

    # Создаем вторую ось Y для скорости на том же X
    ax2_speed = ax2.twinx()
    ax2_speed.plot(t_points, v_x_filtered, label='Скорость X', color='r', linestyle='-')
    ax2_speed.plot(t_points, v_y_filtered, label='Скорость Y', color='m', linestyle='-')
    ax2_speed.set_ylabel('Скорость')
    ax2_speed.legend(loc='center left', bbox_to_anchor=(1.05, 0.8))

    params_str = "\n".join(f"{name}: {val}" for name, val in logs)
    plt.figtext(0.88, 0.8, params_str, fontsize=12, va='top', ha='left', bbox=dict(facecolor='white', alpha=0.7))


    plt.subplots_adjust(right=1.20)
    plt.tight_layout()
    fig.canvas.manager.set_window_title(title)
    plt.show()
