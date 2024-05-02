import rccar

# Real data and simulation:
throttle_real_command = 90

sim_file_directory = "curva_t_255_d_20"
exp_file_directory = "data"
exp_file_name = "hyper_vitesse_ca_tourne.txt"

# ODE Solver parameters - INPUTS FOR EVERY ANALYSIS
abserr = 1.0e-8
relerr = 1.0e-6
initial_time = 2

# Getting the real data value and the time date to make the simulation
treal, tsim, stoptime, numpoints, xreal, yreal, vreal, areal, t_max, length, t0, Xe0, Ye0, v0, a0, psi0_tout_droit = rccar.read_exp_file(exp_file_directory, exp_file_name, initial_time)
# Variable Parameter values
mi = 0.59
C_s = 0.16
C_alpha = 0.9

# Fixed Parameter values
max_steer_angle = 30.0
m = 0.04
Iz = 0.000266
lf = 0.047
lr = 0.05
Lw = 0.0
r = 0.024
Fz = m*9.98/4
throttle2omega = 0.059/r
# throttle2omega = 0.0595/r

# Simulation conditions
initial_time_throttle = 0
final_time_throttle = 4
throttle_type = "step"
delta_real_command = 180
initial_time_delta = 0.5
final_time_delta = 1.5 
delta_type = "step"

# Initial conditions

x0 = Xe0
y0 = Ye0 
# psi0 = psi0_tout_droit + rccar.np.pi # in rad
psi0 = 0
xp0 = 0.0
xpp0 = 0.0
yp0 = 0.0
ypp0 = 0.0
psip0 = 0.0
psipp0 = 0.0
var_delta = 0.0
val_0 = [x0, y0, psi0, xp0, xpp0, yp0, ypp0, psip0, psipp0, Xe0, Ye0, var_delta]

# Packaging the parameters
#t = rccar.np.linspace(0,stoptime,numpoints)
ode_param = [abserr, relerr, stoptime, numpoints]
throttle_sim = -(throttle_real_command - 127)
throttle_parameters = rccar.set_throttle(throttle_sim, initial_time_throttle, final_time_throttle, throttle_type)
print(throttle_parameters)
delta_parameters = rccar.set_delta(delta_real_command, initial_time_delta, final_time_delta, delta_type)
print(delta_parameters)
param = [max_steer_angle, m, Iz, lf, lr, Lw, r, mi, C_s, C_alpha, Fz, throttle2omega, throttle_parameters, delta_parameters]
print(val_0)
voiture = rccar.NonLinearBycicle(sim_file_directory, param, val_0)
voiture.run(tsim, ode_param)

tsimu, xsimu, xpsimu, ysimu, ypsimu, psi, psip, Xe, Ye,  xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv = rccar.read_sim_file(sim_file_directory)

# PLOTS
rccar.ComparisonPlot(treal, xreal, yreal, vreal, tsim, Xe, Ye, xpsimu, ypsimu, tv, dv, exp_file_name)




