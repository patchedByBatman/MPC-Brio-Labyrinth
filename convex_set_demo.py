from labyrinth import Labyrinth
from animator import Animator
from matplotlib.patches import Circle
from polytope_helper import get_patch
from ray_tracer import RayTracer, ConvexSetConstructor
from solution_path import SolutionPath
from matplotlib.animation import FuncAnimation


lb = Labyrinth()
lb.build_labyrinth()   
an = Animator()  
rt = RayTracer(starting_point=(6, 9))
csc = ConvexSetConstructor()

sp = SolutionPath()
path = sp.path
poly_patches = []
ball_patches = []
ins = []
scatters = [1]
for point in path:
    ball_patches.append(Circle(point, 0.5, fc="red"))
    rt.update_rays(point)
    intersections = rt.compute_ray_intersections(lb.walls + lb.holes)
    poly = csc.get_convex_set(rt.starting_point, intersections)
    ins.append(intersections)
    poly_patch = get_patch(poly, color='lightblue')
    poly_patches.append(poly_patch)

scatters[0] = an.pos_ax.scatter([intersection[1][0] for intersection in intersections], [intersection[1][1] for intersection in intersections])
def update_animation(frame):
    scatters[0].remove()
    scatters[0] = an.pos_ax.scatter([inter[1][0] for inter in ins[frame]], [inter[1][1] for inter in ins[frame]])
    an.pos_ax.patches[-1].remove()
    an.pos_ax.patches[-1].remove()
    poly_patch = poly_patches[frame]
    ball_patch = ball_patches[frame]
    poly_patch = an.pos_ax.add_patch(poly_patch)
    ball_patch = an.pos_ax.add_patch(ball_patch)
    return poly_patch, ball_patch
    
animation = FuncAnimation(fig=an.fig, func=update_animation, frames=range(len(ball_patches)), interval=2000)
an.build_labyrinth()
animation.save('manual_ball_positioning_feasible_convex_sets.mp4', writer='ffmpeg', fps=30, bitrate=1800, dpi=400)
an.show_animation()
