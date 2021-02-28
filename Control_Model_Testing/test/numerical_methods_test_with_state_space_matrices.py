# -*- coding: utf-8 -*-
# Import required modules
# --------------------------------------------------------------------------- #
from numpy import pi
from numpy import arange
import matplotlib.pyplot as plt
# --------------------------------------------------------------------------- #


# Functions
# --------------------------------------------------------------------------- #
def solve_xdot_rk_4(A, B, x_store_0, x_store_1, u, dt, steps):
    '''
    Solves the state-space equations using a fourth order Runge-Kutta method.
    '''

    # Initialise variables
    x = [0, 0]
    x_dot = [0, 0]

    # K1 = dt*(A*x+B*u)
    K1 = dt*(A[0][0]*x_store_0[steps] +
             A[0][1]*x_store_1[steps] + B[0]*u[0])
    # K2 = dt*(A*(x+K1/2)+B*u)
    K2 = dt*(A[0][0]*(x_store_0[steps]+K1/2) +
             A[0][1]*(x_store_1[steps]+K1/2) + B[0]*u[0])
    # K3 = dt*(A*(x+K2/2)+B*u)
    K3 = dt*(A[0][0]*(x_store_0[steps]+K2/2) +
             A[0][1]*(x_store_1[steps]+K2/2) + B[0]*u[0])
    # K4 = dt*(A*(x+K3)+B*u)
    K4 = dt*(A[0][0]*(x_store_0[steps]+K3) +
             A[0][1]*(x_store_1[steps]+K3) + B[0]*u[0])

    # Calculate the derivative of x with respect to time
    x_dot[0] = (K1 + (2.0*K2) + (2.0*K3) + K4)/6.0
    # Calculate x
    x[0] = x_store_0[steps] + x_dot[0]

    # K1 = dt*(A*x+B*u)
    K1 = dt*(A[1][0]*x_store_0[steps] +
             A[1][1]*x_store_1[steps] + B[1]*u[0])
    # K2 = dt*(A*(x+K1/2)+B*u)
    K2 = dt*(A[1][0]*(x_store_0[steps]+K1/2) +
             A[1][1]*(x_store_1[steps]+K1/2) + B[1]*u[0])
    # K3 = dt*(A*(x+K2/2)+B*u)
    K3 = dt*(A[1][0]*(x_store_0[steps]+K2/2) +
             A[1][1]*(x_store_1[steps]+K2/2) + B[1]*u[0])
    # K4 = dt*(A*(x+K3)+B*u)
    K4 = dt*(A[1][0]*(x_store_0[steps]+K3) +
             A[1][1]*(x_store_1[steps]+K3) + B[1]*u[0])

    # Calculate the derivative of x with respect to time
    x_dot[1] = (K1 + (2.0*K2) + (2.0*K3) + K4)/6.0
    # Calculate x
    x[1] = x_store_1[steps] + x_dot[1]

    return(x)


def solve_xdot_f_euler(A, B, x_store_0, x_store_1, u, dt, steps):
    '''
    Solves the state-space equations using the forward Euler method.
    '''

    # Initialise variables
    x = [0, 0]

    # Calculate x
    x[0] = x_store_0[steps] + dt*(A[0][0]*x_store_0[steps] +
                                  A[0][1]*x_store_1[steps] + B[0]*u[0])
    x[1] = x_store_1[steps] + dt*(A[1][0]*x_store_0[steps] +
                                  A[1][1]*x_store_1[steps] + B[1]*u[0])

    return(x)


def solve_xdot_b_euler(A, B, x_store_0, x_store_1, u, dt, steps):
    '''
    Solves the state-space equations using an equation derived from the
    backwards Euler method.
    '''

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
    B_times_u[0] = x_store_0[steps] + (B[0]*u[0])*dt
    B_times_u[1] = x_store_1[steps] + (B[1]*u[0])*dt

    # Calculate x
    x[0] = inv_A[0][0]*B_times_u[0] + inv_A[0][1]*B_times_u[1]
    x[1] = inv_A[1][0]*B_times_u[0] + inv_A[1][1]*B_times_u[1]

    return(x)


def solve_xdot_b_euler_iter(A, B, x_store_0, x_store_1, u, dt, steps):
    '''
    Solves the state-space equations using the backwards Euler method through
    fixed point iteration.
    '''

    # Initialise variables
    N = 0
    x_fixed = [x_store_0[steps], x_store_1[steps]]
    x = [0, 0]
    x_dot = [0, 0]

    # Guess the first step in the iteration by using the forward Euler method
    x = solve_xdot_f_euler(A, B, x_store_0, x_store_1, u, dt, steps)

    # Iterate to calculate x
    while(N < 10):
        x_dot[0] = (A[0][0]*x[0] + A[0][1]*x[1] + B[0]*u[0])
        x_dot[1] = (A[1][0]*x[0] + A[1][1]*x[1] + B[1]*u[0])

        x[0] = x_fixed[0] + x_dot[0]*dt
        x[1] = x_fixed[1] + x_dot[1]*dt

        N += 1

    return(x)


def solve_xdot_b_euler_iter_var(A, B, x_store_0, x_store_1, u, dt, steps,
                                integral, q_from_target, error):
    '''
    Solves the state-space equations using the backwards Euler method through
    fixed point iteration. The u vector is also iterated.
    '''

    # Initialise variables
    N = 0
    x_fixed = [x_store_0[steps], x_store_1[steps]]
    x = [0, 0]
    x_dot = [0, 0]

    # Guess the first step in the iteration by using the forward Euler method
    x = solve_xdot_f_euler(A, B, x_store_0, x_store_1, u, dt, steps)

    # Iterate to calculate x
    while(N < 10):
        error = x[1]
        u_old = u
        u, integral = get_modified_scout_input(q_from_target, error, integral,
                                               dt)
        u[0] = u_old[0] + u[0]*dt

        x_dot[0] = (A[0][0]*x[0] + A[0][1]*x[1] + B[0]*u[0])
        x_dot[1] = (A[1][0]*x[0] + A[1][1]*x[1] + B[1]*u[0])

        x[0] = x_fixed[0] + x_dot[0]*dt
        x[1] = x_fixed[1] + x_dot[1]*dt

        N += 1

    return(x, integral, u)


def f_1_for_newton(A, B, x_1, x_2, x_fixed, u, dt):
    return(x_1 - x_fixed - dt*(A[0][0]*x_1 + A[0][1]*x_2 + B[0]*u))


def f_2_for_newton(A, B, x_1, x_2, x_fixed, u, dt):
    return(x_2 - x_fixed - dt*(A[1][0]*x_1 + A[1][1]*x_2 + B[1]*u))


def solve_xdot_b_euler_newton(A, B, x_store_0, x_store_1, u, dt, steps,
                              integral, q_from_target, error):
    # Initialise variables
    N = 0
    x_fixed = [x_store_0[steps], x_store_1[steps]]
    x = [0, 0]
    inv_J = [[0, 0], [0, 0]]
    h = 1e-6

    # Guess the first step in the iteration by using the forward Euler method
    x = solve_xdot_f_euler(A, B, x_store_0, x_store_1, u, dt, steps)

    # Iterate to calculate x
    while(N < 10):
        error = x[1]
        u_old = u
        u, integral = get_modified_scout_input(q_from_target, error, integral,
                                               dt)
        u[0] = u_old[0] + u[0]*dt

        # Calculate the terms in the Jacobian
        df1_dx1 = ((f_1_for_newton(A, B, x[0]+h, x[1], x_fixed[0], u[0], dt) -
                   f_1_for_newton(A, B, x[0]-h, x[1], x_fixed[0], u[0], dt)) /
                   (2*h))
        df1_dx2 = ((f_1_for_newton(A, B, x[0], x[1]+h, x_fixed[0], u[0], dt) -
                   f_1_for_newton(A, B, x[0], x[1]-h, x_fixed[0], u[0], dt)) /
                   (2*h))
        df2_dx1 = ((f_2_for_newton(A, B, x[0]+h, x[1], x_fixed[1], u[0], dt) -
                   f_2_for_newton(A, B, x[0]-h, x[1], x_fixed[1], u[0], dt)) /
                   (2*h))
        df2_dx2 = ((f_2_for_newton(A, B, x[0], x[1]+h, x_fixed[1], u[0], dt) -
                   f_2_for_newton(A, B, x[0], x[1]-h, x_fixed[1], u[0], dt)) /
                   (2*h))

        # Calculate the inverse of the Jacobian
        inv_det_J = 1 / (df1_dx1*df2_dx2 - df1_dx2*df2_dx1)
        inv_J[0][0] = inv_det_J * df2_dx2
        inv_J[0][1] = inv_det_J * -df1_dx2
        inv_J[1][0] = inv_det_J * -df2_dx1
        inv_J[1][1] = inv_det_J * df1_dx1

        f_1 = f_1_for_newton(A, B, x[0], x[1], x_fixed[0], u[0], dt)
        f_2 = f_2_for_newton(A, B, x[0], x[1], x_fixed[1], u[0], dt)

        x[0] = x[0] - (inv_J[0][0]*f_1 + inv_J[0][1]*f_2)
        x[1] = x[1] - (inv_J[1][0]*f_1 + inv_J[1][1]*f_2)

        N += 1

    return(x, integral, u)


def get_modified_scout_input(q_from_target, error, integral, iteration_time):
    '''
    Gets the input for the modified Scout.
    '''

    # Calculate the output from the feedback sum
    u_from_error_sum = (q_from_target - error)

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
    '''
    Runs and plots the results of the state-space systems.
    '''

    # State-space matrices for the Scout and target aircraft
    A_sc = [[-3.36272133719075, 53.0090457250976],
            [-1.22585753557309, -5.71920679668875]]
    B_sc = [-13.4847622573906, -38.384583022159]
    A_t = [[-0.295319442940517, 177.255749314106],
           [-0.0104482212700093, -0.448358567032906]]
    B_t = [-6.2938910678574, -4.88848338612147]
    u = [0]
    x = [0, 0]

    # Initialise arrays for storing the results of the simulation
    x_store_0_sc = [0]
    x_store_1_sc = [0]
    x_store_0_t = [0]
    x_store_1_t = [0]
    x_store_0_sc_mod = [0]
    x_store_1_sc_mod = [0]
    u_store_modified_scout = [0]

    times = []

    # Initialise the value of the integral used for the PD controller
    integral = 0

    # Simulation time settings
    dt = 1/100
    sim_time = 15
    steps = 0

    # Run the simulation
    while(steps < sim_time/dt):
        if steps == 0:
            u[0] = 15 * pi/180

        # Solve the state-space equations for the target
        x = solve_xdot_b_euler(A_t, B_t, x_store_0_t, x_store_1_t, u, dt,
                               steps)
        x_store_0_t.append(x[0])
        x_store_1_t.append(x[1])

        # Solve the state-space equations for the Scout
        x = solve_xdot_b_euler(A_sc, B_sc, x_store_0_sc, x_store_1_sc, u, dt,
                               steps)

        x_store_0_sc.append(x[0])
        x_store_1_sc.append(x[1])

        # Get the input into the modified Scout for the first timestep
        if steps == 0:
            u_modified_scout, integral = get_modified_scout_input(
                    x_store_1_t[steps], x_store_1_sc_mod[steps], integral, dt)

        # Solve the state-space equations for the modified Scout
        x, integral, u_modified_scout = solve_xdot_b_euler_newton(
                               A_sc, B_sc, x_store_0_sc_mod, x_store_1_sc_mod,
                               u_modified_scout, dt, steps, integral,
                               x_store_1_t[steps], x_store_1_sc_mod[steps])
        x_store_0_sc_mod.append(x[0])
        x_store_1_sc_mod.append(x[1])
        u_store_modified_scout.append(u_modified_scout[0])

        times.append(dt*steps)

        steps += 1

    # Plot the results of the simulation
    # Plot pitch rates
    fig, axs = plt.subplots(3, 1)
    axs[0].plot(times, x_store_1_sc[:-1], 'k', label='Scout')
    axs[0].plot(times, x_store_1_t[:-1], 'b', label='Target')
    axs[0].plot(times, x_store_1_sc_mod[:-1], 'r--', label='Modified Scout')
    axs[0].legend(loc='lower right')
    axs[0].set_title("Pitch Rate vs Time")
    axs[0].set_ylabel(r"$q$ ($rad/s$)")
    axs[0].set_xlim(0, sim_time)
    axs[0].set_xticks(arange(0, sim_time+1, step=1))
    axs[0].grid()
    # Plot vertical velocities
    axs[1].plot(times, x_store_0_sc[:-1], 'k', label='Scout')
    axs[1].plot(times, x_store_0_t[:-1], 'b', label='Target')
    axs[1].plot(times, x_store_0_sc_mod[:-1], 'r--', label='Modified Scout')
    axs[1].legend(loc='lower right')
    axs[1].set_title("Vertical Velocity vs Time")
    axs[1].set_ylabel(r"$w$ ($m/s$)")
    axs[1].set_xlim(0, sim_time)
    axs[1].set_xticks(arange(0, sim_time+1, step=1))
    axs[1].grid()

    # Plot control inputs
    axs[2].plot(times, u_store_modified_scout[:-1], 'k')
    axs[2].set_title(r"$u$ into Modified Scout vs Time")
    axs[2].set_ylabel(r"$u$ ($rad$)")
    axs[2].set_xlim(0, sim_time)
    axs[2].set_xticks(arange(0, sim_time+1, step=1))
    axs[2].grid()

    # Place labels on the x-axis of the subplots
    for ax in axs.flat:
        ax.set(xlabel=r'Time ($s$)')

    # Adjust the horizontal spacing between subplots
    plt.subplots_adjust(hspace=0.5)

    print("Scout: ", x_store_1_sc[0:4])
    print("Target: ", x_store_1_t[0:4])
    print("U into modified Scout: ", u_store_modified_scout[0:4])
    print("Modified Scout: ", x_store_1_sc_mod[0:4])
# --------------------------------------------------------------------------- #


# Run the simulation
# --------------------------------------------------------------------------- #
run_sim()
# --------------------------------------------------------------------------- #
