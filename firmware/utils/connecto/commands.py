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

    def zero_move_xy_test(self, *args, confirmation_callback=None, **kwargs):
        self._packet.new_packet(1)
        self._packet.add_command(2, 1, 0, 0, 1) # включить моторы и обнулить машинные координаты (flag4)
        # self._packet.add_command(1, 0,1,0,0,1000,2000,100,200)
        self._packet.add_command(1, 1,1,0,0,50000,100000,20,10)
        self._packet.add_command(1, 1,1,0,0,1000,2000,500,250)
        self._packet.add_command(1, 0,0,0,0,1000,2000,500,250)
        self._packet.add_command(1, 0,0,0,0,50000,100000,20,10)
        # self._packet.add_command(1, 0,1,0,0,1000,2000,-100,-200)

        return self._connector.send_command(self._packet, confirmation_callback=confirmation_callback)

