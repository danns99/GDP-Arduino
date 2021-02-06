# -*- coding: utf-8 -*-
# Import required modules
# --------------------------------------------------------------------------- #
import matplotlib.pyplot as plt
# --------------------------------------------------------------------------- #


# Functions
# --------------------------------------------------------------------------- #
def solve_xdot_f_euler(A, B, x_store_0, x_store_1, u, dt, steps):
    B_times_u = [0, 0]
    x = [0, 0]

    B_times_u[0] = (B[0]*0 + B[1]*u[0])
    B_times_u[1] = (B[0]*0 + B[1]*u[0])

    x[0] = x_store_0[steps] + dt*(A[0][0]*x_store_0[steps] + A[0][1]*x_store_1[steps] + B_times_u[0])
    x[1] = x_store_1[steps] + dt*(A[1][0]*x_store_0[steps] + A[1][1]*x_store_1[steps] + B_times_u[1])

    return(x)


def solve_xdot_b_euler(A, B, x_store_0, x_store_1, u, dt, steps):
    B_times_u = [0, 0]
    inv_A = [[0, 0],[0, 0]]
    x = [0, 0]

    inv_det_A = 1 / (((1-(A[0][0]*dt))*(1-(A[1][1]*dt))) - ((-A[0][1]*dt)*((-A[1][0]*dt))))
    inv_A[0][0] = inv_det_A * (1-(A[1][1]*dt))
    inv_A[0][1] = inv_det_A * (A[0][1]*dt)
    inv_A[1][0] = inv_det_A * (A[1][0]*dt)
    inv_A[1][1] = inv_det_A * (1-(A[0][0]*dt))

    B_times_u[0] = x_store_0[steps] + (B[0]*0 + B[1]*u[0])*dt
    B_times_u[1] = x_store_1[steps] + (B[0]*0 + B[1]*u[0])*dt

    x[0] = inv_A[0][0]*B_times_u[0] + inv_A[0][1]*B_times_u[1]
    x[1] = inv_A[1][0]*B_times_u[0] + inv_A[1][1]*B_times_u[1]

    return(x)


def run_sim():
    A_sc = [[-3.36272133719075, 53.0090457250976],[-1.22585753557309, -5.71920679668875]]
    B_sc = [-13.4847622573906, -38.384583022159]
    A_t = [[-0.295319442940517, 177.255749314106],[-0.0104482212700093, -0.448358567032906]]
    B_t = [-6.2938910678574, -4.88848338612147]
    u = [0, 0]
    x = [0, 0]

#    x_store_0_euler = [0]
#    x_store_1_euler = [0]

    x_store_0_sc_b_euler = [0]
    x_store_1_sc_b_euler = [0]
    x_store_0_t_b_euler = [0]
    x_store_1_t_b_euler = [0]

    times = []

    dt = 0.01
    sim_time = 15
    steps = 0

    while(steps<sim_time/dt):
        if steps == 0:
            u[0] = 15 * 3.1415/180
        
        # Solve state-space equations for scout
        x = solve_xdot_b_euler(A_sc, B_sc, x_store_0_sc_b_euler, x_store_1_sc_b_euler, u, dt, steps)
        x_store_0_sc_b_euler.append(x[0])
        x_store_1_sc_b_euler.append(x[1])
        # Solve state-space equations for target
        x = solve_xdot_b_euler(A_t, B_t, x_store_0_t_b_euler, x_store_1_t_b_euler, u, dt, steps)
        x_store_0_t_b_euler.append(x[0])
        x_store_1_t_b_euler.append(x[1])

#        x = solve_xdot_f_euler(A, B, x_store_0_euler, x_store_1_euler, u, dt, steps)
#        x_store_0_euler.append(x[0])
#        x_store_1_euler.append(x[1])

        times.append(dt*steps)

        steps += 1

#    plt.plot(times, x_store_1[:-1], label='Forward Euler')
    plt.plot(times, x_store_1_sc_b_euler[:-1], 'k', label='Scout Backwards Euler')
    plt.plot(times, x_store_1_t_b_euler[:-1], 'r', label='Target Backwards Euler')
    plt.legend(loc='upper right')
    plt.xlim(0, sim_time)
# --------------------------------------------------------------------------- #
    
# Run simulation
# --------------------------------------------------------------------------- #
run_sim()    
# --------------------------------------------------------------------------- #
