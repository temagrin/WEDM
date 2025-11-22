
import matplotlib.pyplot as plt



if __name__ == '__main__':
    time = [0, 1, 2, 3, 4, 5]
    displacement = [10, 15, 13, 17, 16, 18]   # перемещение
    speed = [7, 10, 14, 16, 15, 19]           # скорость
    force = [5, 9, 12, 15, 14, 20]             # сила

    fig, ax1 = plt.subplots()

    # Основная ось Y - перемещение
    ax1.plot(time, displacement, 'b-', label='Перемещение')
    ax1.set_xlabel('Время')
    ax1.set_ylabel('Перемещение', color='b')
    ax1.tick_params(axis='y', labelcolor='b')

    # Вторая ось Y - скорость
    ax2 = ax1.twinx()
    ax2.plot(time, speed, 'r-', label='Скорость')
    ax2.set_ylabel('Скорость', color='r')
    ax2.tick_params(axis='y', labelcolor='r')

    # Третья ось Y - сила, размещена с правым смещением
    # Для третьей оси нужно использовать дополнительный трюк с Spines
    ax3 = ax1.twinx()
    ax3.spines['right'].set_position(('outward', 60))  # смещение вправо
    ax3.plot(time, force, 'g-', label='Сила')
    ax3.set_ylabel('Сила', color='g')
    ax3.tick_params(axis='y', labelcolor='g')

    # Можно добавить легенду с объединением всех линий
    lines = ax1.get_lines() + ax2.get_lines() + ax3.get_lines()
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, loc='upper left')

    plt.title('Графики с разными единицами измерения и общей осью X (время)')
    plt.show()