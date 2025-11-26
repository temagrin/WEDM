from connector import Command, Connector


class CommandHelper:
    def __init__(self, connector: Connector):
        self._connector = connector
        self._command = Command()

    # List of implemented commands
    def zero_command(self, *args, **kwargs):
        self._command.set_attr(command_id=0)
        return self._connector.send_command(self._command, None)

    def set_power_motor_command(self, motor_xy_enable, motor_a_enable, motor_b_enable, answer_callback=None):
        self._command.set_attr(
            command_id=1,
            flag1=motor_xy_enable,
            flag2=motor_a_enable,
            flag3=motor_b_enable
        )
        return self._connector.send_command(self._command, answer_callback)

    def set_speed_command(self, speed_x: int, speed_y: int, speed_a: int, speed_b: int, answer_callback=None):
        self._command.set_attr(
            command_id=2,
            flag1=speed_x > 0,
            flag2=speed_y > 0,
            flag3=speed_a > 0,
            flag4=speed_b > 0,
            param1=speed_x,
            param2=speed_y,
            param3=speed_a,
            param4=speed_b,
        )
        return self._connector.send_command(self._command, answer_callback)
