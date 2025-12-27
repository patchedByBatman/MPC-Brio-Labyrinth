from labyrinth import Labyrinth

from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle, Circle
import numpy as np


class Animator(Labyrinth):
    def __init__(self, animation_name="bRiO Labyrinth"):
        super().__init__()
        self.animation_name = animation_name
        self.fig = plt.figure(num=animation_name, figsize=(16, 9), layout="constrained", dpi=100)
        self.grid = self.fig.add_gridspec(9, 16)
        self.fig.tight_layout()
        self.walls = []
        self.holes = []
        self.pos_ref = Circle((0, 0), 0.1, fc="red")

        self.pos_ax = self.fig.add_subplot(self.grid[0:6, 0:6])
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

        self.vel_ax = self.fig.add_subplot(self.grid[0:2, 7:16])
        self.vx_mes_line, = self.vel_ax.plot([], [], color='red')
        self.vy_mes_line, = self.vel_ax.plot([], [], color='green')
        self.vx_est_line, = self.vel_ax.plot([], [], color='brown')
        self.vy_est_line, = self.vel_ax.plot([], [], color='violet')
        self.vx_line, = self.vel_ax.plot([], [], color='blue')
        self.vy_line, = self.vel_ax.plot([], [], color='orange')
        self.vel_ax.grid()
        self.vel_ax.minorticks_on()
        self.vel_ax.set_xlabel("time (s)")
        self.vel_ax.set_ylabel("Velocity (m/s)")
        self.vel_ax_xlims = [0, 90]
        self.vel_ax_ylims = [-2.5, 2.5]
        self.vel_ax.set_xlim(self.vel_ax_xlims[0], self.vel_ax_xlims[1])
        self.vel_ax.set_ylim(self.vel_ax_ylims[0], self.vel_ax_ylims[1])
        self.vel_ax.legend(["x-velocity-mes", "y-velocity-mes", "x-velocity-est", "y-velocity-est", "x-velocity", "y-velocity"])

        self.a_b_ax = self.fig.add_subplot(self.grid[3:6, 7:16])
        self.alpha_mes_line, = self.a_b_ax.plot([], [], color='red')
        self.beta_mes_line, = self.a_b_ax.plot([], [], color='green')
        self.alpha_line, = self.a_b_ax.plot([], [], color="blue")
        self.beta_line, = self.a_b_ax.plot([], [], color='orange')
        self.a_b_ax.grid()
        self.a_b_ax.minorticks_on()
        self.a_b_ax.set_xlabel("time (s)")
        self.a_b_ax.set_ylabel("Tilt angles (deg)")
        self.a_b_ax_xlims = [0, 90]
        self.a_b_ax_ylims = [-np.pi/12, np.pi/12]
        self.a_b_ax.set_xlim(self.a_b_ax_xlims[0], self.a_b_ax_xlims[1])
        self.a_b_ax.set_ylim(self.a_b_ax_ylims[0], self.a_b_ax_ylims[1])
        self.a_b_ax.legend(["alpha-mes", "beta-mes", "alpha", "beta"])

        self.x_y_ax = self.fig.add_subplot(self.grid[7:9, 0:7])
        self.x_mes_line, = self.x_y_ax.plot([], [], color='red')
        self.y_mes_line, = self.x_y_ax.plot([], [], color="green")
        self.x_est_line, = self.x_y_ax.plot([], [], color="brown")
        self.y_est_line, = self.x_y_ax.plot([], [], color="violet")
        self.x_line, = self.x_y_ax.plot([], [], color='blue')
        self.y_line, = self.x_y_ax.plot([], [], color='orange')
        self.x_y_ax.grid()
        self.x_y_ax.minorticks_on()
        self.x_y_ax.set_xlabel("time (s)")
        self.x_y_ax.set_ylabel("Position (m)")
        self.x_y_ax_xlims = [0, 90]
        self.x_y_ax_ylims = [-12, 12]
        self.x_y_ax.set_xlim(self.x_y_ax_xlims[0], self.x_y_ax_xlims[1])
        self.x_y_ax.set_ylim(self.x_y_ax_ylims[0], self.x_y_ax_ylims[1])
        self.x_y_ax.legend(["x-position-mes", "y-position-mes", "x-position-est", "y-position-est", "x-position", "y-position"])

        self.g_ax = self.fig.add_subplot(self.grid[7:9, 8:16])
        self.g_line, = self.g_ax.plot([], [])
        self.g_ax.grid()
        self.g_ax.minorticks_on()
        self.g_ax.set_xlabel("time (s)")
        self.g_ax.set_ylabel("g estimate (m/s^2)")
        self.g_ax_xlims = [0, 90]
        self.g_ax_ylims = [-10, 2]
        self.g_ax.set_xlim(self.g_ax_xlims[0], self.g_ax_xlims[1])
        self.g_ax.set_ylim(self.g_ax_ylims[0], self.g_ax_ylims[1])
        self.g_ax.legend(["g estimate"])


        
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

if __name__ == "__main__":
    an = Animator()
    an.build_labyrinth()