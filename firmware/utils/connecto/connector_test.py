from time import sleep, time

from connector import Connector, Status
from commands import CommandHelper
from color_CLI import *


def default_answer_callback(status: Status):
    pass
    # print("-=Answer callback=-")
    # status.print_status()


def make_command(func, **kwargs):
    return func, kwargs


def main():
    connector = Connector(port_name="/dev/ttyACM0")
    # connector = Connector(port_name="COM20")
    connector.connect()

    cmd = CommandHelper(connector)
    sleep(1)

    commands = [
        make_command(cmd.zero_move_xy_test, confirmation_callback=default_answer_callback),
        # make_command(cmd.zero_move_xy_test2, confirmation_callback=default_answer_callback),
    ]

    last_command_time = time()
    command_interval = 2

    c = 0
    for i in range(1200000000):
        connector.process()
        current_time = time()
        if (current_time - last_command_time) >= command_interval:
            last_command_time = current_time
            if c<len(commands):
                func, kwargs = commands[c]
                func(**kwargs)
                c+=1
            print(f"[{connector.status.seq_id}]::{CYAN}{connector.status.current_position_x}|{connector.status.current_position_y}{RESET}")
    connector.disconnect()


if __name__ == "__main__":
    main()
