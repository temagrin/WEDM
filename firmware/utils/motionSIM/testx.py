from matplotlib import pyplot as plt


class TestAccelerator:
    def __init__(self):
        self.current_position = 0
        self.elasted_dist = 17000
        self.current_speed = 0
        self.max_speed = 52600
        self.finish_speed = 0
        self.acceleration_factor = 1
        self.deceleration_factor = 1
        self.step_interval = 0
        self.last_step_time = 0


    def speed_tick(self):
        left = self.elasted_dist * 2 * self.deceleration_factor
        right = (self.current_speed**2 - self.finish_speed ** 2) * 0.00002  # delta_t фиксированная
        if left <= right:
            self.current_speed -= self.deceleration_factor
            if self.current_speed < self.finish_speed:
                self.current_speed = self.finish_speed

        elif self.current_speed<=self.max_speed:
            self.current_speed += self.acceleration_factor

        if self.current_speed ==0:
            self.step_interval = 0
        else:
            self.step_interval = 1_000_000 / abs(self.current_speed)


    def position_tick(self, t):
        if t>=(self.last_step_time + self.step_interval):
            self.last_step_time = t
            if self.elasted_dist>0:
                self.elasted_dist -=1
                self.current_position+=1


if __name__ == '__main__':
    machine = TestAccelerator()
    t = 0
    t_poinst = []
    p_poinst = []
    s_poinst = []
    while machine.elasted_dist:
        t+=1
        machine.position_tick(t) # каждую микросекунду
        if t % 20 == 0 : # каждые 20 микросекунд
            machine.speed_tick()
        p_poinst.append(machine.current_position)
        s_poinst.append(machine.current_speed)
        t_poinst.append(t)

    fig, ax1 = plt.subplots()
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (steps)', color='tab:blue')
    ax1.plot(t_poinst, p_poinst, color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax2 = ax1.twinx()
    ax2.set_ylabel('Speed (steps/s)', color='tab:red')
    ax2.plot(t_poinst, s_poinst, color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')
    plt.show()