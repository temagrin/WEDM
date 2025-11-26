import wx
import serial.tools.list_ports

class MainFrame(wx.Frame):
    def __init__(self):
        super().__init__(None, title="Serial Port GUI", size=(600, 400))

        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Комбо для выбора порта
        hbox_ports = wx.BoxSizer(wx.HORIZONTAL)
        self.port_choice = wx.Choice(panel, choices=self.get_serial_ports())
        hbox_ports.Add(wx.StaticText(panel, label="Select Port:"), 0, wx.ALL | wx.CENTER, 5)
        hbox_ports.Add(self.port_choice, 1, wx.ALL | wx.EXPAND, 5)
        main_sizer.Add(hbox_ports, 0, wx.EXPAND)

        # Кнопки подключить/отключить
        hbox_buttons = wx.BoxSizer(wx.HORIZONTAL)
        self.btn_connect = wx.Button(panel, label="Connect")
        self.btn_disconnect = wx.Button(panel, label="Disconnect")
        self.btn_disconnect.Disable()
        hbox_buttons.Add(self.btn_connect, 1, wx.ALL, 5)
        hbox_buttons.Add(self.btn_disconnect, 1, wx.ALL, 5)
        main_sizer.Add(hbox_buttons, 0, wx.EXPAND)

        # Поле вывода (многострочное)
        self.text_output = wx.TextCtrl(panel, style=wx.TE_MULTILINE|wx.TE_READONLY)
        main_sizer.Add(self.text_output, 1, wx.ALL | wx.EXPAND, 5)

        # Поле ввода команды + кнопка Отправить
        hbox_command = wx.BoxSizer(wx.HORIZONTAL)
        self.text_command = wx.TextCtrl(panel)
        self.btn_send = wx.Button(panel, label="Send")
        hbox_command.Add(self.text_command, 1, wx.ALL | wx.EXPAND, 5)
        hbox_command.Add(self.btn_send, 0, wx.ALL, 5)
        main_sizer.Add(hbox_command, 0, wx.EXPAND)

        panel.SetSizer(main_sizer)

        # Инициализация serial как None
        self.serial_port = None

        # Связываем события с функциями
        self.btn_connect.Bind(wx.EVT_BUTTON, self.on_connect)
        self.btn_disconnect.Bind(wx.EVT_BUTTON, self.on_disconnect)
        self.btn_send.Bind(wx.EVT_BUTTON, self.on_send)

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def on_connect(self, event):
        port = self.port_choice.GetStringSelection()
        if not port:
            wx.MessageBox("Please select a port", "Error", wx.ICON_ERROR)
            return
        try:
            import serial
            self.serial_port = serial.Serial(port, 9600, timeout=0.5)
            self.text_output.AppendText(f"Connected to {port}\n")
            self.btn_connect.Disable()
            self.btn_disconnect.Enable()
        except Exception as e:
            wx.MessageBox(f"Failed to open port: {e}", "Error", wx.ICON_ERROR)

    def on_disconnect(self, event):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.text_output.AppendText("Disconnected\n")
        self.serial_port = None
        self.btn_connect.Enable()
        self.btn_disconnect.Disable()

    def on_send(self, event):
        if self.serial_port and self.serial_port.is_open:
            cmd = self.text_command.GetValue()
            if cmd:
                try:
                    self.serial_port.write(cmd.encode('utf-8') + b'\n')
                    self.text_output.AppendText(f"Sent: {cmd}\n")
                    self.text_command.Clear()
                except Exception as e:
                    wx.MessageBox(f"Failed to send command: {e}", "Error", wx.ICON_ERROR)
        else:
            wx.MessageBox("Not connected to any port", "Error", wx.ICON_ERROR)

class MyApp(wx.App):
    def OnInit(self):
        frame = MainFrame()
        frame.Show()
        return True

if __name__ == "__main__":
    app = MyApp()
    app.MainLoop()