from labyrinth import Labyrinth

from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle, Circle


class Animator(Labyrinth):
    def __init__(self, animation_name="bRiO Labyrinth"):
        super().__init__()
        self.animation_name = animation_name
        self.fig = plt.figure(num=animation_name, figsize=(8, 6), layout="constrained", dpi=100)
        self.grid = self.fig.add_gridspec(9, 16)
        self.walls = []
        self.holes = []
        self.pos_ref = Circle((0, 0), 0.1, fc="red")

        self.pos_ax = self.fig.add_subplot(self.grid[0:-1, 0:-1])
        self.pos_ax.plot([], [])
        self.pos_ax.set_aspect('equal')
        self.pos_ax.grid()
        self.pos_ax.minorticks_on()
        self.pos_ax.set_xlabel("x-position (m)")
        self.pos_ax.set_ylabel("y-position (m)")
        self.pos_ax_xlims = [-10, 10]
        self.pos_ax_ylims = [-10, 10]
        self.pos_ax.set_xlim(min(self.pos_ax_xlims) - 1, max(self.pos_ax_xlims) + 1)
        self.pos_ax.set_ylim(min(self.pos_ax_ylims) - 1, max(self.pos_ax_ylims) + 1)

        # self.vel_ax = self.fig.add_subplot(grid[0:2, 7:16])
        # self.vel_ax.grid()
        # self.vel_ax.set_xlim(time[0], time[-1])
        # self.vel_ax.set_ylim(np.min(state_sequence[:, 3:5]) - 0.2, np.max(state_sequence[:, 3:5]) + 0.2)

        # self.acc_brk_ax = self.fig.add_subplot(grid[3:6, 7:16])
        # self.acc_brk_ax.set_xlim(time[0], time[-1])
        # self.acc_brk_ax.grid()
        # self.acc_brk_ax.set_ylim(0 - 0.1, 1.1 + 0.1)

        # self.omega_ax = self.fig.add_subplot(grid[7:9, 0:7])
        # self.omega_ax.grid()
        # self.omega_ax.set_xlim(time[0], time[-1])
        # self.omega_ax.set_ylim(np.min(state_sequence[:, 5]) - 0.2, np.max(state_sequence[:, 5]) + 0.2)

        # self.delta_ax = self.fig.add_subplot(grid[7:9, 8:16])
        # self.delta_ax.grid()
        # self.delta_ax.set_xlim(time[0], time[-1])
        # self.delta_ax.set_ylim(np.min(input_sequence[:, 1]) - 0.2, np.max(input_sequence[:, 1]) + 0.2)


        
        # self.vx_line, = self.vel_ax.plot([], [])
        # self.vy_line, = self.vel_ax.plot([], [])
        # self.vel_ax.legend(["$v_x(t)$", "$v_y(t)$"])
        # self.vel_ax.set_xlabel("time (s)")
        # self.vel_ax.set_ylabel("Local velocity (m/s)")
        # self.omega_line, = self.omega_ax.plot([], [])
        # self.omega_ax.legend(["$\omega(t)$"])
        # self.omega_ax.set_xlabel("time (s)")
        # self.omega_ax.set_ylabel("Omega (rad/s)")

        # self.acc_line, = self.acc_brk_ax.plot([], [])
        # self.delta_line, = self.delta_ax.plot([], [])
        # self.delta_ax.legend(["$\delta(t)$"])
        # self.delta_ax.set_xlabel("time (s)")
        # self.delta_ax.set_ylabel("Delta (rad)")
        # self.brake_line, = self.acc_brk_ax.plot([], [])
        # self.acc_brk_ax.legend(["PWM(t)", "Brake"])
        # self.acc_brk_ax.set_xlabel("time (s)")
        # self.acc_brk_ax.set_ylabel("Controls (normalised)")

    def add_wall(self, xy, width, height):
        self.walls.append(Rectangle(xy, width, height, fc="black"))
    
    def add_hole(self, xy, radius):
        self.walls.append(Circle(xy, radius, fc="black"))

    def update_pos_ref(self, center_xy):
        self.pos_ref.set_center(center_xy)

    def build_labyrinth(self):
        super().build_labyrinth()
        self.pos_ax.add_patch(self.pos_ref)
        for wall in self.walls:
            self.pos_ax.add_patch(wall)
        for hole in self.holes:
            self.pos_ax.add_patch(hole)
    
    def show_animation(self):
        plt.show()


an = Animator()
an.build_labyrinth()