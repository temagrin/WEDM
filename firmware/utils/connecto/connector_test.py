from time import sleep, time

from connector import Connector, Status
from commands import CommandHelper


def default_answer_callback(status: Status):
    print("-=Answer callback=-")
    print(status.ack_mask)
    status.print_status()


def make_command(func, **kwargs):
    return func, kwargs


def main():
    connector = Connector(port_name="/dev/ttyACM0")
    # connector = Connector(port_name="COM20")
    connector.connect()

    cmd = CommandHelper(connector)
    sleep(1)

    commands = [
        make_command(cmd.zero_command, confirmation_callback=default_answer_callback),
        make_command(cmd.zero_command1, confirmation_callback=default_answer_callback),
        make_command(cmd.zero_command2, confirmation_callback=default_answer_callback),
        make_command(cmd.zero_command3, confirmation_callback=default_answer_callback),
    ]

    last_command_time = time()
    command_interval = 3
    i = 0
    while i < len(commands):
        current_time = time()
        if current_time - last_command_time >= command_interval and commands:
            last_command_time = current_time
            func, kwargs = commands[i]
            if func(**kwargs):
                i += 1
        connector.process()

    for i in range(120):
        connector.process()
    connector.disconnect()


if __name__ == "__main__":
    main()
