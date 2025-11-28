from connector import Packet, Connector


class CommandHelper:
    def __init__(self, connector: Connector):
        self._connector = connector
        self._packet = Packet()

    # List of implemented commands
    def zero_command(self, *args, confirmation_callback=None, **kwargs):
        self._packet.new_packet(0)
        self._packet.add_command(1)
        return self._connector.send_command(self._packet, confirmation_callback=confirmation_callback)

    def zero_command1(self, *args, confirmation_callback=None, **kwargs):
        self._packet.new_packet(1)
        self._packet.add_command(1)
        return self._connector.send_command(self._packet, confirmation_callback=confirmation_callback)

    def zero_command2(self, *args, confirmation_callback=None, **kwargs):
        self._packet.new_packet(2)
        self._packet.add_command(1)
        self._packet.add_command(2)
        self._packet.add_command(3)

        return self._connector.send_command(self._packet, confirmation_callback=confirmation_callback)

    def zero_command3(self, *args, confirmation_callback=None, **kwargs):
        self._packet.new_packet(3)
        self._packet.add_command(1)
        self._packet.add_command(2)
        self._packet.add_command(1)
        self._packet.add_command(2)

        return self._connector.send_command(self._packet, confirmation_callback=confirmation_callback)
