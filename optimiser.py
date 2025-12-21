import numpy as np
import opengen as og
import casadi.casadi as cs
from dynamics import Dynamics
from functools import partial

# NMPC parameter configuration
nz = 6  # number of states
nu = 2  # number of inputs
N = 50  # prediction horizon

# Bicycle model dynamics
Ts = 0.001  # sampling time
dyn = Dynamics(Ts=Ts)

# State and input boundaries
zmin = [-12, -np.inf, -12, -np.inf, -np.pi/12, -np.pi/12]
zmax = [12, -np.inf, 12, -np.inf, np.pi/12, np.pi/12]
umin_seq = [-np.pi/24, -np.pi/24] * N
umax_seq = [np.pi/24, np.pi/24] * N

# NMPC penalty weights
Q = np.diagflat([1000, 10, 1000, 10, 10, 10])
R = np.diagflat([600, 600])
R2 = 100

# NMPC formulation state and input parameter sequences
u_seq = cs.SX.sym("u_seq", N * nu, 1)
problem_params = cs.SX.sym("state_params", 2*nz + 4, 1)  # [z_0, z_ref, xmin, xmax, ymin, ymax]

# NMPC initial settings
z_0 = problem_params[ : nz]  # initial state
z_ref = problem_params[nz : 2 * nz]  # reference destination
position_constraint_params = problem_params[2 * nz:]

# define the stage cost
def stage_cost(z, u):
    return (z.T - z_ref.T) @ Q @ (z - z_ref) + u.T @ R @ u

# define the terminal cost
def terminal_cost(z):
    return (z.T - z_ref.T) @ Q @ (z - z_ref)

# initialise constraints
z_t = z_0
total_cost = 0
position_constraints = []

# formulate the NMPC total cost and constraints
for t in range(N):
    u_current = u_seq[t * nu : (t + 1) * nu]  # set current time step input
    total_cost += stage_cost(z_t, u_current)  # add stage cost to total cost for the current time step
    position_constraints += [
        z_t[0][0] - position_constraint_params[0],   # x >= xmin
        position_constraint_params[1] - z_t[0][0],   # x <= xmax
        z_t[2][0] - position_constraint_params[2],   # y >= ymin
        position_constraint_params[3] - z_t[2][0]    # y <= ymax
    ]

    # position_constraints += cs.fmin(0, position_constraint_params[0] - z_t[0][0])
    # # total_cost += z_t[0][0] <= position_constraint_params[1]
    # # total_cost += position_constraint_params[2] <= z_t[2][0]
    # # total_cost += z_t[2][0] <= position_constraint_params[3]
    z_t = dyn.linear_dynamics_dt_opt(z_t, u_current)  # set current state to resultant state of the current input
# add terminal cost to total cost
total_cost += terminal_cost(z_t)

# define rectangular constraints for inputs
U = og.constraints.Rectangle(umin_seq, umax_seq)
position_constraints = cs.vertcat(*position_constraints)
# define and build the problem
problem = og.builder.Problem(u_seq, problem_params, total_cost)
problem = problem.with_constraints(U)
problem = problem.with_penalty_constraints(position_constraints)

build_config = og.config.BuildConfiguration() \
    .with_build_directory("optimizer") \
    .with_tcp_interface_config()

meta = og.config.OptimizerMeta().with_optimizer_name("brio_labyrinth")

solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-4)\
    .with_penalty_weight_update_factor(3)

builder = []
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config).with_verbosity_level(0)

builder.build()