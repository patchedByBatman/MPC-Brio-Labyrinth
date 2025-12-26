## MPC Brio Labyrinth

Solving Brio labyrinth using MPC. A paper I recently read, titled _Adaptive Nonlinear Model Predictive Control for a Real-World Labyrinth Game_ [1] by Johannes Gaber et.al., inspired this project.

In [1], the authors have used 2 MPCs, one high-level and one low-level, to solve the labyrinth. The high-level controller generates optimal trajectories for the low-level controller, which then controls the ball to follow the computed trajectory.

In the paper, the high-level controller takes into account all the wall and hole constraints for computing the optimal trajectory. The constraints make the space non-convex, and because the dynamics of the system are non-linear, finding solutions for the formulated optimal control problem is computationally heavy.

In this project, to avoid computational complexity, instead of using a 2-level controller architecture and taking non-convex constraints, I intend to compute a local feasible convex set around the ball's current position at every instant and use this as the constraint set.

The goal at each iteration would be to make the ball reach the furthest point (on the solution path towards the goal) in the computed convex set. The Idea is to combine this with linearised dynamics, which would yield a convex optimisation problem, which could be solved efficiently.

We approach the project's development in 2 stages.
1. To develop an initial framework to compute the convex sets as the ball navigates through the labyrinth.
2. To formulate the MPC and solve the labyrinth.

## Stage 1: Initial framework to compute feasible convex sets.
This work is done, and a demo script is provided in [_convex_set_demo.py_](convex_set_demo.py) file. Below is a demo gif (it has been slowed down for demonstration, and doesn't reflect the actual computational time).

![Demo of feasible convex sets computation](manual_ball_positioning_feasible_convex_sets.gif)

In the GIF above, the red circle represents the ball, and the light blue colour polygon represents the computed convex set at each ball position throughout the labyrinth.


## Stage 2: Simulation of the setup.
## Part A: Simulation using Linear dynamics.
Usage: Run [_optimiser.py_](optimiser.py) to build the MPC formulation. Then run [_main.py_](main.py) to simulate the setup. There should be an mp4 media file generated under the name _Linear\_dyn\_simulation\_with\_feasible\_convex\_sets.mp4_ with the simulation results. 

In part A of stage 2, only the linearized dynamics of the labyrinth were used to simulate the MCP at work. Below is a GIF showcasing the results of the simulation. The MCP was able to solve the (20m x 20m) labyrinth in 90 seconds of simulation time.
![Demo of MPC solving the labyrinth in a simulation environment using linearized dynamics](Linear_dyn_simulation_with_feasible_convex_sets.gif)

In the above GIF, the larger red circle represents the steel ball, the smaller red circle represents position reference for the MPC, and the collection blue dots represents the solution path of the labyrinth.

As it can be seen from the above GIF, the MPC successfully solves the labyrinth. But this simulation is based on the linearized dynamics of the labyrinth setup. The next step is to integrate the position Kalman Filter and then simulate the non-linear dynamics. 

## References
[1] J. Gaber, T. Bi and R. D’Andrea, "Adaptive Nonlinear Model Predictive Control for a Real-World Labyrinth Game," 2024 IEEE 63rd Conference on Decision and Control (CDC), Milan, Italy, 2024, pp. 7478-7483, doi: 10.1109/CDC56724.2024.10886880. keywords: {Adaptation models;Computational modeling;Games;Predictive models;Real-time systems;Robustness;Trajectory;Springs;Optimization;Predictive control},

