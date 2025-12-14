from labyrinth import Labyrinth
from animator import Animator
from matplotlib.patches import Circle
from polytope_helper import get_patch
from ray_tracer import RayTracer, ConvexSetConstructor
from solution_path import SolutionPath



lb = Labyrinth()
lb.build_labyrinth()   
an = Animator()  
rt = RayTracer(starting_point=(6, 9))
csc = ConvexSetConstructor()

sp = SolutionPath()
path = sp.path
poly_patches = []
ball_patches = []
for point in path:
    ball_patches.append(Circle(point, 0.5, fc="red"))
    rt.update_rays(point)
    intersections = rt.compute_ray_intersections(lb.walls + lb.holes)
    # an.pos_ax.scatter([intersection[1][0] for intersection in intersections], [intersection[1][1] for intersection in intersections])
    poly = csc.get_convex_set(rt.starting_point, intersections)
    # poly.plot(an.pos_ax, color='lightblue')
    poly_patch = get_patch(poly, color='lightblue')
    poly_patches.append(poly_patch)

# an.pos_ax.scatter([intersection[1][0] for intersection in intersections], [intersection[1][1] for intersection in intersections])

# an.pos_ax.add_patch(poly_patches[0])
# an.pos_ax.patches[-1].remove()

def update_animation(frame):
    # an.pos_ax.collections.clear()
    an.pos_ax.patches[-1].remove()
    an.pos_ax.patches[-1].remove()
    poly_patch = poly_patches[frame]
    ball_patch = ball_patches[frame]
    poly_patch = an.pos_ax.add_patch(poly_patch)
    ball_patch = an.pos_ax.add_patch(ball_patch)
    # poly_patch = poly.plot(an.pos_ax)
    return poly_patch, ball_patch
    
# an.pos_ax.scatter([intersection[1][0] for intersection in intersections], [intersection[1][1] for intersection in intersections])
# print(intersections)
# an.pos_ax.clear()
from matplotlib.animation import FuncAnimation
animation = FuncAnimation(fig=an.fig, func=update_animation, frames=range(len(ball_patches)), interval=500)
an.build_labyrinth()
animation.save('manual_ball_positioning_feasible_convex_sets.mp4', writer='ffmpeg', fps=30, bitrate=1800, dpi=400)
from matplotlib import pyplot as plt
plt.show()
# an.fig.show()
