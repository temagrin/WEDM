from time import sleep, time

from connector import Connector, Status
from commands import CommandHelper


def default_answer_callback(status: Status):
    print("-=Answer callback=-")
    print(f"\tlast_cammand_uid={status.last_command_uid}")
    if status.last_command_uid == 5:
        print(time())
        exit(0)


def make_command(func, **kwargs):
    return func, kwargs


def main():
    connector = Connector(port_name="/dev/ttyACM0")
    connector.connect()

    cmd = CommandHelper(connector)
    sleep(1)

    commands = [
        make_command(cmd.set_power_motor_command, motor_xy_enable=True, motor_a_enable=True, motor_b_enable=True,
                     answer_callback=default_answer_callback),
        make_command(cmd.set_speed_command, speed_x=50000, speed_y=10000, speed_a=-5000, speed_b=-10000,
                     answer_callback=default_answer_callback),
        make_command(cmd.set_speed_command, speed_x=-50000, speed_y=-10000, speed_a=10000, speed_b=5000,
                     answer_callback=default_answer_callback),
        make_command(cmd.set_speed_command, speed_x=0, speed_y=0, speed_a=10000, speed_b=5000,
                     answer_callback=default_answer_callback),
        make_command(cmd.set_power_motor_command, motor_xy_enable=False, motor_a_enable=False, motor_b_enable=False,
                     answer_callback=default_answer_callback),
    ]

    last_command_time = time()
    command_interval = 5
    i = 0
    while i < len(commands):
        current_time = time()
        if current_time - last_command_time >= command_interval and commands:
            last_command_time = current_time
            func, kwargs = commands[i]
            if func(**kwargs):
                i += 1
        connector.process()
    print(time())
    while True:
        connector.process()
        # exit(0) в колбеке default_answer_callback когда придет подтверждение задачи c uid == 5
        # connector.disconnect()


if __name__ == "__main__":
    main()
