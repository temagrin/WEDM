from cases import TestCaseBase, ALL_CASES
from plotter import plot_points
from fake_motor_controller import MotorController
from fake_motor_controller2 import MotorController2


def test_scenario(test_case: TestCaseBase):
    motor_controller = MotorController2()
    current_time = 0
    test_paths = test_case.test_paths[:]
    x_points = [0, ]
    y_points = [0, ]
    t_points = [0, ]
    steps_total = 0
    complite_text = "Да"
    count_fsm_ticks = 0
    while True:
        if (not test_paths and motor_controller.ready()) or (current_time > test_case.max_test_time):
            stop_time = current_time
            if (current_time > test_case.max_test_time):
                complite_text = "Нет"
            finish_x = motor_controller.current_x_position
            finish_y = motor_controller.current_y_position
            break
        if motor_controller.ready() and test_paths:
            new_task = test_paths.pop(0)
            if test_paths:
                next_task = test_paths[0]
                k = motor_controller.look_ahead(new_task[0], new_task[1], next_task[0], next_task[1])
                max_allowed_speed = min(test_case.max_speed, test_case.max_speed)
                junction_speed = int(max_allowed_speed * k)
            else:
                junction_speed = 0
            motor_controller.move_to(new_task[0], new_task[1], test_case.max_speed, junction_speed)
        if motor_controller.tick(current_time):
            steps_total += 1
        if (current_time % test_case.delta_t_planer) == 0:
            motor_controller.fsm_tick()
            count_fsm_ticks += 1

        x_points.append(motor_controller.current_x_position)
        y_points.append(motor_controller.current_y_position)
        t_points.append(current_time)
        current_time += test_case.delta_t

    logs= [
        ("Время движения", f"{round((stop_time/1000000),3)}сек"),
        ("Дельта T", f"{test_case.delta_t}мкс"),
        ("Тики планера", f"{test_case.delta_t_planer}мкс"),
        ("Тиков планера", count_fsm_ticks),
        ("Заданая скорость", f"{test_case.max_speed}ш/с"),
        ("Тиков с шагом", steps_total),
        ("Финиш X", finish_x),
        ("Финиш Y", finish_y),
        ("Успел все?", complite_text)
    ]

    plot_points(test_case.title, logs, x_points, y_points, t_points)


if __name__ == '__main__':
    for test_case in ALL_CASES:
        test_scenario(test_case())