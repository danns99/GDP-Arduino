# -*- coding: utf-8 -*-
# Import required modules
# --------------------------------------------------------------------------- #
from numpy import pi
from numpy import arange
import matplotlib.pyplot as plt
# --------------------------------------------------------------------------- #


# Functions
# --------------------------------------------------------------------------- #
def solve_xdot_f_euler(A, B, x_store_0, x_store_1, u, dt, steps):
    # Initialise variables
    x = [0, 0]

    # Calculate B*u
    B_times_u = (B[0]*0 + B[1]*u[0])

    # Calculate x
    x[0] = x_store_0[steps] + dt*(A[0][0]*x_store_0[steps] +
                                  A[0][1]*x_store_1[steps] + B_times_u)
    x[1] = x_store_1[steps] + dt*(A[1][0]*x_store_0[steps] +
                                  A[1][1]*x_store_1[steps] + B_times_u)

    return(x)


def solve_xdot_b_euler(A, B, x_store_0, x_store_1, u, dt, steps):
    # Initialise variables
    B_times_u = [0, 0]
    inv_A = [[0, 0], [0, 0]]
    x = [0, 0]

    # Calculate 1 over the determinant of the I-A matrix
    inv_det_A = 1 / (((1-(A[0][0]*dt))*(1-(A[1][1]*dt))) -
                     ((-A[0][1]*dt)*((-A[1][0]*dt))))
    inv_A[0][0] = inv_det_A * (1-(A[1][1]*dt))
    inv_A[0][1] = inv_det_A * (A[0][1]*dt)
    inv_A[1][0] = inv_det_A * (A[1][0]*dt)
    inv_A[1][1] = inv_det_A * (1-(A[0][0]*dt))

    # Calculate B*u
    B_times_u[0] = x_store_0[steps] + (B[0]*0 + B[1]*u[0])*dt
    B_times_u[1] = x_store_1[steps] + (B[0]*0 + B[1]*u[0])*dt

    # Calculate x
    x[0] = inv_A[0][0]*B_times_u[0] + inv_A[0][1]*B_times_u[1]
    x[1] = inv_A[1][0]*B_times_u[0] + inv_A[1][1]*B_times_u[1]

    return(x)


def get_modified_scout_input(q_from_target, error, integral, iteration_time):
    # Calculate the output from the feedback sum
    u_from_error_sum = q_from_target - error

    # Values for the PD controller
    N = 100.0
    K_p = -86.717674561182620
    K_d = 0.637114873357555

    # Calculate the PD terms
    proportional_term = K_p*u_from_error_sum
    derivative_term = ((K_d*u_from_error_sum) - integral) * N

    # Update the value of the integral
    integral += iteration_time * derivative_term

    return([proportional_term + derivative_term], integral)


def run_sim():
    # State-space matrices for the Scout and target aircraft
    A_sc = [[-3.36272133719075, 53.0090457250976],
            [-1.22585753557309, -5.71920679668875]]
    B_sc = [-13.4847622573906, -38.384583022159]
    A_t = [[-0.295319442940517, 177.255749314106],
           [-0.0104482212700093, -0.448358567032906]]
    B_t = [-6.2938910678574, -4.88848338612147]
    u = [0, 0]
    x = [0, 0]

    # Initialise arrays for storing the results of the simulation
    x_store_0_sc_b_euler = [0]
    x_store_1_sc_b_euler = [0]
    x_store_0_t_b_euler = [0]
    x_store_1_t_b_euler = [0]
    x_store_0_sc_mod_b_euler = [0]
    x_store_1_sc_mod_b_euler = [0]
    u_store_modified_scout = [0]

    times = []

    # Initialise the value of the integral used for the PD controller
    integral = 0

    # Simulation time settings
    dt = 0.001
    sim_time = 15
    steps = 0

    # Run the simulation
    while(steps < sim_time/dt):
        if steps == 0:
            u[0] = 15 * pi/180

        # Solve the state-space equations for the Scout
        x = solve_xdot_b_euler(A_sc, B_sc, x_store_0_sc_b_euler,
                               x_store_1_sc_b_euler, u, dt, steps)
        x_store_0_sc_b_euler.append(x[0])
        x_store_1_sc_b_euler.append(x[1])

        # Solve the state-space equations for the target
        x = solve_xdot_b_euler(A_t, B_t, x_store_0_t_b_euler,
                               x_store_1_t_b_euler, u, dt, steps)
        x_store_0_t_b_euler.append(x[0])
        x_store_1_t_b_euler.append(x[1])

        # Get the input into the modified Scout
        u_modified_scout, integral = get_modified_scout_input(
                x_store_1_t_b_euler[steps], x_store_1_sc_mod_b_euler[steps],
                integral, dt)

        # Solve the state-space equations for the modified Scout
        x = solve_xdot_b_euler(A_sc, B_sc, x_store_0_sc_mod_b_euler,
                               x_store_1_sc_mod_b_euler, u_modified_scout,
                               dt, steps)
        x_store_0_sc_mod_b_euler.append(x[0])
        x_store_1_sc_mod_b_euler.append(x[1])
        u_store_modified_scout.append(u_modified_scout[0])

        times.append(dt*steps)

        steps += 1

    # Plot the results of the simulation
    # Plot pitch rates
    fig, axs = plt.subplots(2, 1)
    axs[0].plot(times, x_store_1_sc_b_euler[:-1], 'k',
             label='Scout Backwards Euler')
    axs[0].plot(times, x_store_1_t_b_euler[:-1], 'b',
             label='Target Backwards Euler')
    axs[0].plot(times, x_store_1_sc_mod_b_euler[:-1], 'r--',
             label='Modified Scout Backwards Euler')
    axs[0].legend(loc='upper right')
    axs[0].set_ylabel(r"$q$ ($rad/s$)")
    axs[0].set_title("Pitch Rate vs Time")
    axs[0].set_xticks(arange(0, sim_time+1, step=1))
    axs[0].set_yticks(arange(-0.9, 0.2, step=0.1))
    axs[0].grid()
    axs[0].set_xlim(0, sim_time)
    axs[0].set_ylim(-0.9, 0.1)

    # Plot control inputs
    axs[1].plot(times, u_store_modified_scout[:-1], 'k')
    axs[1].set_xticks(arange(0, sim_time+1, step=1))
    axs[1].set_xlim(0, sim_time)
    axs[1].set_ylim(min(u_store_modified_scout), max(u_store_modified_scout))
    axs[1].set_ylabel(r"$u$ ($rad$)")
    axs[1].set_title(r"$u$ into Modified Scout vs Time")
    axs[1].grid()

    # Place labels on the x-axis of the subplots
    for ax in axs.flat:
        ax.set(xlabel=r'time ($s$)')

    # Adjust the horizontal spacing between subplots
    plt.subplots_adjust(hspace=0.5)
# --------------------------------------------------------------------------- #


# Run the simulation
# --------------------------------------------------------------------------- #
run_sim()
# --------------------------------------------------------------------------- #
