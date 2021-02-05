# -*- coding: utf-8 -*-
import numpy as np
from scipy.optimize import root
import matplotlib.pyplot as plt

times = np.linspace(0,0.75,100)
dt = times[1] - times[0]
y_0 = 1.3
y = [y_0]
b_euler_fixed_iter_vals = [y_0]
b_euler_newton_vals = [y_0]
b_euler_newton_secant_vals = [y_0]
b_euler_root_finder_vals = [y_0]
num_grad = []
act_ans = []
act_grad = []

def func(t):
    return(1.3*np.exp(-9*t))

def func_dt(y):
    return(-9*y)

def b_euler_fixed_iter(t, dt, y):
    N=0
    y_fixed = y
    y_dot = func_dt(y)
    y = y + y_dot*dt
    while(N<10):
        y_dot = func_dt(y)
        y = y_fixed + y_dot*dt
        N+=1
    return(y)

def b_euler_newton(t, dt, y):
    N=0
    y_dot = func_dt(y)
    y_n = y + y_dot*dt
    while(N<10):
        y_n_iter = y_n - ((y-y_n+(dt*func_dt(y_n))) / (-1 + dt*y_n/-9))
        y_n = y_n_iter
        N+=1
    return(y_n, (-1 + dt*y_n/-9))

def b_euler_newton_secant(dt, y_n_old, y_n_old_old, y_n_old_old_old):
    N=0
    y_dot = func_dt(y_n_old)
    y_n = y_n_old + y_dot*dt
    while(N<10):
        #f_prime = (y_n_old - y_n_old_old) / (dt)
        f_prime = -((y_n_old_old-y_n_old+(dt*func_dt(y_n_old))) - (y_n_old_old_old-y_n_old_old+(dt*func_dt(y_n_old_old)))) / (dt)
        y_n_iter = y_n - ((y_n_old-y_n+(dt*func_dt(y_n))) / (f_prime))
        y_n = y_n_iter
        N+=1
    return(y_n, f_prime)

def opt_func(y_n, y, dt):
    return(y-y_n+dt*func_dt(y_n))

def b_euler_root_finder(dt, y_n_old):
    y_dot = func_dt(y_n_old)
    y_n = y_n_old + y_dot*dt
    y_n = root(opt_func, y_n,  args=(y,dt))
    return(y_n.x)


for i in range(len(times)):
    y = b_euler_fixed_iter_vals[i]
    b_euler_fixed_iter_vals.append(b_euler_fixed_iter(times[i], dt, y))

    y = b_euler_newton_vals[i]
    ans, grad = b_euler_newton(times[i], dt, y)
    b_euler_newton_vals.append(ans)
    act_grad.append(grad)
    
    if i>0:
        y = b_euler_newton_secant_vals[i]
        y_old = b_euler_newton_secant_vals[i-1]
        y_old_old = 0# b_euler_newton_secant_vals[i-2]
        ans, grad = b_euler_newton_secant(dt, y, y_old, y_old_old)
        b_euler_newton_secant_vals.append(ans)
        num_grad.append(grad)
    if i==0:
        b_euler_newton_secant_vals.append(y_0 + func_dt(y_0)*dt)
    
    y = b_euler_root_finder_vals[i]
    b_euler_root_finder_vals.append(b_euler_root_finder(dt, y))

    act_ans.append(func(times[i]))

plt.plot(times, act_ans, label='analytic')
plt.plot(times, b_euler_fixed_iter_vals[:-1], '--', label='b_euler fixed-point iter')
plt.plot(times, b_euler_newton_vals[:-1], '.', label='b_euler newton')
plt.plot(times, b_euler_root_finder_vals[:-1], '*', label='b_euler newton root')
#plt.plot(times, b_euler_newton_secant_vals[:-1], '*', label='b_euler newton-secant')
plt.legend(loc='upper right')

#plt.plot(times, act_grad)
#plt.plot(times[:-1], num_grad)
#plt.xlim(0,0.3)
#plt.ylim(-10,-1.000) #-1.001,-1.000