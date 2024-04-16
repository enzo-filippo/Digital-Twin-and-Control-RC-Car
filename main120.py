import rccar
import real_data


sim_file_directory = "curva_t_255_d_20"
exp_file_directory = "data"
exp_file_name = "120.txt"

# ODE Solver parameters - INPUTS FOR EVERY ANALYSIS
abserr = 1.0e-8
relerr = 1.0e-6
initial_time = 3

# Getting the real data value and the time date to make the simulation
treal, tsim, stoptime, numpoints, xreal, yreal, vreal, areal, t_max, length, t0, x0, y0, v0, a0 = rccar.read_exp_file(exp_file_directory, exp_file_name, initial_time)

#stoptime = 12.8-starttime
#numpoints = 130
#step = numpoints/stoptime

# Parameter values
max_steer_angle = 30.0
m = 0.04
Iz = 0.000266
lf = 0.047
lr = 0.05
Lw = 0.0
r = 0.024
mi = 0.57
C_s = 0.9
C_alpha = 0.9
Fz = m*9.98/4
throttle2omega = 0.05166/r

# Simulation conditions
throttle_real_command = 120
initial_time_throttle = 0
final_time_throttle = 100
throttle_type = "step"
delta_real_command = 0
initial_time_delta = 0
final_time_delta = stoptime
delta_type = "straight"

# Initial conditions
#x0 = 0.0 # real values that comes from the experimental
#y0 = 0.0 # real values that comes from the experimental
psi0 = -5.187103785879343
xp0 = 0.0
xpp0 = 0.0
yp0 = 0.0
ypp0 = 0.0
psip0 = 0.0
psipp0 = 0.0
Xe0 = 3.6077931899999998
Ye0 = 1.3031706500000002
var_delta = 0.0
val_0 = [x0, y0, psi0, xp0, xpp0, yp0, ypp0, psip0, psipp0, Xe0, Ye0, var_delta]

# Packaging the parameters
#t = rccar.np.linspace(0,stoptime,numpoints)
ode_param = [abserr, relerr, stoptime, numpoints]
throttle_sim = -(throttle_real_command - 127)
throttle_parameters = rccar.set_throttle(throttle_sim, initial_time_throttle, final_time_throttle, throttle_type)
delta_parameters = rccar.set_throttle(delta_real_command, initial_time_delta, final_time_delta, delta_type)
param = [max_steer_angle, m, Iz, lf, lr, Lw, r, mi, C_s, C_alpha, Fz, throttle2omega, throttle_parameters, delta_parameters]
voiture = rccar.NonLinearBycicle(sim_file_directory, param, val_0)
voiture.run(tsim, ode_param)
t, x, xp, y, yp, psi, psip, Xe, Ye, Xef, Yef, Xer, Yer = rccar.read_sim_file(sim_file_directory)


real_data.plt.figure(figsize=(6, 4.5))
real_data.plt.xlabel("y [m]")
real_data.plt.ylabel("x [m]")
real_data.plt.grid(True)
real_data.plt.axis('equal')
lw = 1
real_data.plt.plot(xreal, yreal,'r:', label ="real")
real_data.plt.plot(Xe, Ye, 'b:', label ="sim")
real_data.plt.legend()
real_data.plt.show()




