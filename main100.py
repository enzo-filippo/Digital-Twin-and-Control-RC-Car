import model
import real_data
import numpy as np

# ODE Solver parameters
abserr = 1.0e-8
relerr = 1.0e-6
starttime = 4.8
stoptime = 7.9-starttime
numpoints = 81
step = numpoints/stoptime
t = model.np.linspace(0,stoptime,numpoints)
ode_param = [abserr, relerr, stoptime, numpoints]

# Parameter values
max_steer_angle = 30.0
m = 0.04
Iz = 0.000266
lf = 0.047
lr = 0.05
Lw = 0.0
r = 0.024
mi = 0.7
C_s = 0.9
C_alpha = 0.9
Fz = m*9.98/4
min_v = 0.1
throttlereal = 100
throttle = -(throttlereal - 127)
throttle2omega = 0.05166/r
param = [max_steer_angle, m, Iz, lf, lr, Lw, r, mi, C_s, C_alpha, Fz, min_v, throttle2omega, throttle]

# Initial conditions
x0 = 0.0
y0 = 0.0
psi0 = model.np.radians(-27.667+180)
xp0 = 0.0 
xpp0 = 0.0
yp0 = 0.0
ypp0 = 0.0
psip0 = 0.0
psipp0 = 0.0
Xe0 = 3.4285081
Ye0 = 0.1956809
var_delta = 0.0
val_0 = [x0, y0, psi0, xp0, xpp0, yp0, ypp0, psip0, psipp0, Xe0, Ye0, var_delta]

voiture = model.NonLinearBycicle("curva_t_255_d_20", param, val_0)
x, y, xp, yp  = voiture.run(t, ode_param)
# voiture.plot()
v = np.sqrt((xp**2 + yp**2))
# voiture.anim(step)


# Example usage

def find_closest_value_position(arr, value):
    absolute_diff = np.abs(arr - value)
    closest_index = np.argmin(absolute_diff)
    return closest_index

treal, xreal, yreal, vreal, areal = real_data.run(throttlereal)
t_initial_real = treal[-1]- t[-1]

position = find_closest_value_position(treal, t_initial_real)
#position = np.where(treal == t_initial_real)[0]
treal += -t_initial_real

real_data.plt.figure(figsize=(6, 4.5))
real_data.plt.xlabel("y [m]")
real_data.plt.ylabel("x [m]")
real_data.plt.grid(True)
real_data.plt.axis('equal')
lw = 1
real_data.plt.plot(xreal, yreal,'r:', label ="real")
real_data.plt.plot(x, y, 'b:', label ="sim")
real_data.plt.legend()
real_data.plt.show()




real_data.plt.figure(figsize=(6, 4.5))
real_data.plt.xlabel("t [s]")
real_data.plt.ylabel("v [m/s]")
real_data.plt.grid(True)
real_data.plt.axis('equal')
real_data.plt.plot(treal[int(position):], vreal[int(position):],'r:', label ="real")
real_data.plt.plot(t, v, 'b:', label ="sim")
real_data.plt.legend()
real_data.plt.show()



