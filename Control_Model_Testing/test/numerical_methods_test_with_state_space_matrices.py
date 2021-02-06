# -*- coding: utf-8 -*-
import matplotlib as plt


def test_matrix():
    A = [[-3.36272133719075, 53.0090457250976],[-1.22585753557309, -5.71920679668875]]
    B = [-13.4847622573906, -38.384583022159]
    u = [0, 0]
    x = [0, 0]

    x_store_0 = [0]
    x_store_1 = [0]

    x_store_0_var = [0]
    x_store_1_var = [0]

    times = []

    inv_A = [[0, 0],[0, 0]]
    B_times_u = [0, 0]
    dt = 0.01
    sim_time = 15
    steps = 0

    while(steps<sim_time/dt):
        if steps == 0:
            u[0] = 15 * 3.1415/180

        inv_det_A = 1 / (((1-(A[0][0]*dt))*(1-(A[1][1]*dt))) - ((-A[0][1]*dt)*((-A[1][0]*dt))))
        inv_A[0][0] = inv_det_A * (1-(A[1][1]*dt))
        inv_A[0][1] = inv_det_A * (A[0][1]*dt)
        inv_A[1][0] = inv_det_A * (A[1][0]*dt)
        inv_A[1][1] = inv_det_A * (1-(A[0][0]*dt))

        B_times_u[0] = x_store_0_var[steps] + (B[0]*0 + B[1]*u[0])*dt
        B_times_u[1] = x_store_1_var[steps] + (B[0]*0 + B[1]*u[0])*dt

        x[0] = inv_A[0][0]*B_times_u[0] + inv_A[0][1]*B_times_u[1]
        x[1] = inv_A[1][0]*B_times_u[0] + inv_A[1][1]*B_times_u[1]

        x_store_0_var.append(x[0])
        x_store_1_var.append(x[1])

        B_times_u[0] = (B[0]*0 + B[1]*u[0])
        B_times_u[1] = (B[0]*0 + B[1]*u[0])

        x[0] = x_store_0[steps] + dt*(A[0][0]*x_store_0[steps] + A[0][1]*x_store_1[steps] + B_times_u[0])
        x[1] = x_store_1[steps] + dt*(A[1][0]*x_store_0[steps] + A[1][1]*x_store_1[steps] + B_times_u[1])

        x_store_0.append(x[0])
        x_store_1.append(x[1])

        times.append(dt*steps)

        steps += 1

    plt.plot(times, x_store_1[:-1], label='Forward Euler')
    plt.plot(times, x_store_1_var[:-1], '--', label='Backwards Euler')
    plt.legend(loc='upper right')

test_matrix()    

