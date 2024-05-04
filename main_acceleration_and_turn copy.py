import rccar2
import animation

# Real data and simulation:
throttle_real_command = 50
delta_real_command = 70

sim_file_directory = "curva_t_255_d_20"
exp_file_directory = "data"
exp_file_name = "hyper_vitesse_ca_tourne.txt"

# ODE Solver parameters - INPUTS FOR EVERY ANALYSIS
abserr = 1.0e-8
relerr = 1.0e-6
initial_time = 2.26

# Getting the real data value and the time date to make the simulation
treal, tsim, stoptime, numpoints, xreal, yreal, vreal, areal, t_max, length, t0, Xe0, Ye0, v0, a0, psi0_tout_droit = rccar2.read_exp_file(exp_file_directory, exp_file_name, initial_time)
# Variable Parameter values
mi = 0.65
C_s = 0.57
C_alpha = 0.22

# Fixed Parameter values
max_steer_angle = 30.0
m = 0.04
Iz = 0.000266
lf = 0.047
lr = 0.05
Lw = 0.0
r = 0.024
Fz = m*9.98/4
throttle2omega = 0.054/r
# throttle2omega = 0.0595/r

# Simulation conditions
initial_time_throttle = 0
final_time_throttle = 1.13
throttle_type = "step"
initial_time_delta = 0.2
final_time_delta = 1.13
delta_type = "step"

# Initial conditions

x0 = Xe0
y0 = Ye0 
# psi0 = psi0_tout_droit + rccar2.np.pi # in rad
psi0 = -rccar2.np.pi/2
xp0 = 0.0
xpp0 = 0.0
yp0 = 0.0
ypp0 = 0.0
psip0 = 0.0
psipp0 = 0.0
var_delta = 0.0
val_0 = [x0, xp0, y0, yp0, psi0, psip0, Xe0, Ye0]

# Packaging the parameters
#t = rccar2.np.linspace(0,stoptime,numpoints)
ode_param = [abserr, relerr, stoptime, numpoints]
throttle_sim = -(throttle_real_command - 127)
throttle_parameters = rccar2.set_throttle(throttle_sim, initial_time_throttle, final_time_throttle, throttle_type)
print(throttle_parameters)
delta_sim = ((delta_real_command-127)*max_steer_angle)/127
delta_parameters = rccar2.set_delta(delta_sim, initial_time_delta, final_time_delta, delta_type)
print(delta_parameters)
param = [max_steer_angle, m, Iz, lf, lr, Lw, r, mi, C_s, C_alpha, Fz, throttle2omega, throttle_parameters, delta_parameters]
print(val_0)
voiture = rccar2.NonLinearBicycle(sim_file_directory, param, val_0)
voiture.run(tsim, ode_param)

tsimu, xsimu, xpsimu, ysimu, ypsimu, psi, psip, Xe, Ye,  xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r = rccar2.read_sim_file(sim_file_directory)


# PLOTS
rccar2.ComparisonPlot(treal, xreal, yreal, vreal, tsim, Xe, Ye, xpsimu, ypsimu, tv, dv, s_f, s_r, exp_file_name)




