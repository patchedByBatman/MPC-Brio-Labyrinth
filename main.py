from animator import Animator
from dynamics import Dynamics
from kf import KalmanFilter
from labyrinth import Labyrinth
from polytope_helper import get_patch
from ray_tracer import RayTracer, ConvexSetConstructor
from solution_path import SolutionPath

import opengen as og
import numpy as np
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation, FFMpegWriter
from polytope import extreme
from scipy.integrate import odeint

mng = og.tcp.OptimizerTcpManager("optimizer/brio_labyrinth")
# Start the TCP server
mng.start()

def distance(p1, p2):
    return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def get_projection_on_path(path, current_pos):
    d1 = np.inf
    idx = None
    p = None
    for i, point in enumerate(path):
        d2 = distance(point, current_pos)
        if d1 > d2:
            d1 = d2
            idx = i
            p = point
    return idx, p, d1

def get_pos_ref(path, poly, current_pos, num_look_points_ahead):
    idx, p, d1 = get_projection_on_path(path, current_pos)
    if idx != None and p != None:
        # if path[idx] in poly:
            # for i in range(num_look_points_ahead):
        look_ahead = idx + num_look_points_ahead #- i
        if look_ahead >= len(path):
            look_ahead = len(path) - 1
                # if path[look_ahead] in poly:
        return path[look_ahead]
    return None

    # for point in path[::-1]:
    #     if point in poly:
    #         return point

def get_pos_bounds(poly):
    ex = extreme(poly)
    xmin = np.min(ex[:, 0])
    xmax = np.max(ex[:, 0])
    ymin = np.min(ex[:, 1])
    ymax = np.max(ex[:, 1])
    return xmin, xmax, ymin, ymax

Ts = 1E-1
sim_time = 100
tsim = np.linspace(0, sim_time, int(sim_time/Ts))
nz = 6
nu = 2
ball_radius = 0.5

an = Animator()  
dyn = Dynamics(Ts=Ts)
kf = KalmanFilter(sampling_frequency=1./Ts, 
                    u0=np.zeros([nu, 1]),
                    x_tilde_0=np.zeros([5, 1]),
                    cache_values=True)
lb = Labyrinth()
lb.build_labyrinth()   
sp = SolutionPath()
path = sp.path
starting_point = path[0]
rt = RayTracer(starting_point=starting_point)
csc = ConvexSetConstructor(ball_radius=ball_radius-0.49)


poly_patches = []
ball_patches = []
intersections_list = []
scatters = [1]
pos_refs = []

z_0 = np.array([path[0][0], 0, path[0][1], 0, 0, 0])
z_current = z_0
state_sequence = np.zeros((tsim.shape[0] + 1, nz))
input_sequence = np.zeros((tsim.shape[0] + 1, nu))
# states_measured = np.random.multivariate_normal(mean=[0.]*nz, cov=np.diagflat([0.00001, 0.00001, 0.00001, 0.00001, np.pi/240000, np.pi/240000]), size=(tsim.shape[0] + 1, )).reshape(tsim.shape[0] + 1, nz)
states_measured = np.random.multivariate_normal(mean=[0.]*nz, cov=np.diagflat([0.0001, 0.0001, 0.0001, 0.0001, np.pi/5000, np.pi/5000]), size=(tsim.shape[0] + 1, )).reshape(tsim.shape[0] + 1, nz)
states_measured[0, :] += np.copy(z_current.reshape((6, )))

for i, t in enumerate(tsim):
    kf_est = kf.update(np.array([states_measured[i][4], states_measured[i][5]]).reshape([nu, 1]), np.array([states_measured[i][0], states_measured[i][2]]).reshape([2, 1]))
    point = (kf_est[0][0], kf_est[2][0])

    rt.update_rays(point)
    intersections = rt.compute_ray_intersections(lb.walls + lb.holes)

    poly = csc.get_convex_set(rt.starting_point, intersections, path, get_projection_on_path(path, point)[0])
    xmin, xmax, ymin, ymax = get_pos_bounds(poly)
    poly_patch = get_patch(poly, color='lightblue')
    pos_ref = get_pos_ref(path, poly, point, 20)
    print(t, point, pos_ref)
    if pos_ref == None:
        pos_ref = point
    z_ref = np.array([pos_ref[0], 0, pos_ref[1], 0, 0, 0]).reshape((6, 1))

    states_with_kf_est = [kf_est[0][0], states_measured[i][1], kf_est[2][0], states_measured[i][3], states_measured[i][4], states_measured[i][5]]
    # pass NMPC problem parameters
    solver_response = mng.call(p=[
            *states_with_kf_est, *z_ref.flatten(),
            xmin, xmax, ymin, ymax
        ])
    assert solver_response.is_ok(), "solver failed!"
    out = solver_response.get()

    # get the set of solutions (N control actions)
    us = out.solution
    # print some useful info for tuning

    # use the first control action from the computed sequence of solutions and simulate one time step
    u_mpc = us[0:nu]
    states, info = odeint(func=dyn.non_linear_dynamics_ct, y0=z_current.reshape((6, )), t=np.linspace(0, Ts, 10), full_output=True, args=(lambda t: u_mpc[0], lambda t: u_mpc[1]))
    z_next = np.array(states[-1]).reshape(nz, 1)
    # cache results
    intersections_list.append(intersections)
    poly_patches.append(poly_patch)
    ball_patches.append(Circle((states[-1][0], states[-1][2]), 0.5, fc="red"))
    pos_refs.append(pos_ref)
    state_sequence[i+1, :] = z_next.T
    input_sequence[i, :] = u_mpc
    states_measured[i+1, :] += z_next.reshape((6, ))
    z_current = z_next.flatten()


# kill the server once simulation in complete
mng.kill()
print(state_sequence)
kf_estimates = np.array(kf.measurement_update_cache()[0]).reshape([-1, 5]).T
# for current_idx_on_path, point in enumerate(path):
#     ball_patches.append(Circle(point, 0.5, fc="red"))
#     rt.update_rays(point)
#     intersections = rt.compute_ray_intersections(lb.walls + lb.holes)
#     poly = csc.get_convex_set(rt.starting_point, intersections, path, current_idx_on_path)
#     intersections_list.append(intersections)
#     poly_patch = get_patch(poly, color='lightblue')
#     poly_patches.append(poly_patch)
an.pos_ax.scatter([point[0] for point in path], [point[1] for point in path], s=1)
# scatters[0] = an.pos_ax.scatter([intersection[1][0] for intersection in intersections], [intersection[1][1] for intersection in intersections])
def update_animation(frame):
    # scatters[0].remove()
    # scatters[0] = an.pos_ax.scatter([inter[1][0] for inter in intersections_list[frame]], [inter[1][1] for inter in intersections_list[frame]])
    an.pos_ax.patches[-1].remove()
    an.pos_ax.patches[-1].remove()
    poly_patch = poly_patches[frame]
    ball_patch = ball_patches[frame]
    poly_patch = an.pos_ax.add_patch(poly_patch)
    ball_patch = an.pos_ax.add_patch(ball_patch)
    an.update_pos_ref(pos_refs[frame])

    t = tsim[0:frame]
    vx = state_sequence[0:frame, 1]
    vy = state_sequence[0:frame, 3]
    vx_mes = states_measured[0:frame, 1]
    vy_mes = states_measured[0:frame, 3]
    vx_est = kf_estimates[1, 0:frame]
    vy_est = kf_estimates[3, 0:frame]
    an.vx_mes_line.set_data(t, vx_mes)
    an.vy_mes_line.set_data(t, vy_mes)
    an.vx_est_line.set_data(t, vx_est)
    an.vy_est_line.set_data(t, vy_est)
    an.vx_line.set_data(t, vx)
    an.vy_line.set_data(t, vy)

    a = state_sequence[0:frame, 4]
    b = state_sequence[0:frame, 5]
    a_mes = states_measured[0:frame, 4]
    b_mes = states_measured[0:frame, 5]
    an.alpha_mes_line.set_data(t, a_mes)
    an.beta_mes_line.set_data(t, b_mes)
    an.alpha_line.set_data(t, a)
    an.beta_line.set_data(t, b)

    x = state_sequence[0:frame, 0]
    y = state_sequence[0:frame, 2]
    x_mes = states_measured[0:frame, 0]
    y_mes = states_measured[0:frame, 2]
    x_est = kf_estimates[0, 0:frame]
    y_est = kf_estimates[2, 0:frame]
    an.x_mes_line.set_data(t, x_mes)
    an.y_mes_line.set_data(t, y_mes)
    an.x_est_line.set_data(t, x_est)
    an.y_est_line.set_data(t, y_est)
    an.x_line.set_data(t, x)
    an.y_line.set_data(t, y)

    an.g_line.set_data(t, kf_estimates[4, 0:frame])

    return poly_patch, ball_patch, an.pos_ref, an.vx_line, an.vy_line, an.vx_mes_line, \
                an.vy_mes_line, an.vx_est_line, an.vy_est_line, an.alpha_line, an.beta_line, \
                an.alpha_mes_line, an.beta_mes_line, an.x_line, an.y_line, an.x_mes_line, \
                an.y_mes_line, an.x_est_line, an.y_est_line, an.g_line
    
animation = FuncAnimation(fig=an.fig, func=update_animation, frames=range(len(ball_patches)), interval=100)
an.build_labyrinth()


# GPU-accelerated FFmpeg writer (NVIDIA NVENC)
# Change 'h264_nvenc' to 'hevc_nvenc' for H.265, or 'h264_qsv' for Intel QuickSync
writer = FFMpegWriter(
    fps=30,
    codec="h264_nvenc",  # NVIDIA GPU encoder
    extra_args=[
        "-preset", "fast",  # speed/quality trade-off
        "-pix_fmt", "yuv420p"  # compatibility
    ]
)

# Save animation
try:
    animation.save('nonlin_sim.mp4', writer=writer)
    print("Animation saved as nonlin_sim.mp4.")
except Exception as e:
    print(f"Error saving animation: {e}")
# animation.save('manual_ball_positioning_feasible_convex_sets2.mp4', writer='ffmpeg', fps=30, bitrate=1800, dpi=400)
an.show_animation()
