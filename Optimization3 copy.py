import rccar2

# Real data and simulation:
throttle_real_command = 105
delta_real_command = 63

sim_file_directory = "curva_t_255_d_20"
exp_file_directory = "data"
exp_file_name = str(throttle_real_command) + "-" + str(delta_real_command) + ".txt"

# ODE Solver parameters - INPUTS FOR EVERY ANALYSIS
abserr = 1.0e-8
relerr = 1.0e-6
initial_time = 1.9

# Getting the real data value and the time date to make the simulation
treal, tsim, stoptime, numpoints, xreal, yreal, vreal, areal, t_max, length, t0, Xe0, Ye0, v0, a0, psi0_tout_droit = rccar2.read_exp_file(exp_file_directory, exp_file_name, initial_time)
# Variable Parameter values
mi = 0.59
C_s = 0.45
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
throttle2omega = 0.05166/r
# throttle2omega = 0.0595/r

# Simulation conditions
initial_time_throttle = 0
final_time_throttle = 100
throttle_type = "step"
initial_time_delta = 2.3
final_time_delta = stoptime
delta_type = "step"

# Initial conditions
x0 = Xe0
y0 = Ye0 
psi0 = rccar2.np.arctan(1.156)+rccar2.np.pi # in rad
xp0 = 0.0
xpp0 = 0.0
yp0 = 0.0
ypp0 = 0.0
psip0 = 0.0
psipp0 = 0.0
#Xe0 = 3.6077931899999998 # real values that comes from the experimental
#Ye0 = 1.3031706500000002 # real values that comes from the experimental
var_delta = 0.0
val_0 = [x0, xp0, y0, yp0, psi0, psip0, Xe0, Ye0]

# Packaging the parameters
#t = rccar2.np.linspace(0,stoptime,numpoints)
ode_param = [abserr, relerr, stoptime, numpoints]
throttle_sim = -(throttle_real_command - 127)
throttle_parameters = rccar2.set_throttle(throttle_sim, initial_time_throttle, final_time_throttle, throttle_type)
print(throttle_parameters)
delta_parameters = rccar2.set_delta(delta_real_command, initial_time_delta, final_time_delta, delta_type)
print(delta_parameters)
param = [max_steer_angle, m, Iz, lf, lr, Lw, r, mi, C_s, C_alpha, Fz, throttle2omega, throttle_parameters, delta_parameters]
print(val_0)
voiture = rccar2.NonLinearBicycle(sim_file_directory, param, val_0)
voiture.run(tsim, ode_param)


C_alpha_vector = rccar2.np.linspace(0.1, 0.9, 10)
C_s_vector = rccar2.np.linspace(0.1, 0.9, 10)
sum_x = rccar2.np.zeros((len(C_alpha_vector),len(C_s_vector)))
sum_y = rccar2.np.zeros((len(C_alpha_vector),len(C_s_vector)))
sum_v = rccar2.np.zeros((len(C_alpha_vector),len(C_s_vector)))
min_v = 10000
min_x = 10000
min_y = 10000
iteracao = 0
for i in range(len(C_alpha_vector)):
    for j in range(len(C_s_vector)):
        iteracao = iteracao +1
        print("iteracao: ", iteracao, " i: ", i, " j: ", j)
        param = [max_steer_angle, m, Iz, lf, lr, Lw, r, mi, C_s_vector[j], C_alpha_vector[i], Fz, throttle2omega, throttle_parameters, delta_parameters]
        voiture = rccar2.NonLinearBicycle(sim_file_directory, param, val_0)
        voiture.run(tsim, ode_param)
        tsimu, xsimu, xpsimu, ysimu, ypsimu, psi, psip, Xe, Ye,  xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r = rccar2.read_sim_file(sim_file_directory)

        # _, min_x_iterado = rccar2.difference(Xe, xreal)
        # if min_x > min_x_iterado:
        #     min_x = min_x_iterado
        #     min_x_i = i
        # _, min_y_iterado = rccar2.difference(Ye, yreal)
        # if min_y > min_y_iterado:
        #     min_y = min_y_iterado
        #     min_y_i = i
        # _, min_v_iterado = rccar2.difference(rccar2.np.sqrt(xpsimu[:-2]**2+ypsimu[:-2]**2), vreal[1:])
        # if min_v > min_v_iterado:
        #     min_v = min_v_iterado
        #     min_v_i = i 
        _, sum_x[i,j] = rccar2.difference(Xe, xreal)
        _, sum_y[i,j] = rccar2.difference(Ye, yreal)
        _, sum_v[i,j] = rccar2.difference(rccar2.np.sqrt(xpsimu**2+ypsimu**2)[:-2], vreal[1:])

Xi, Xj = rccar2.np.unravel_index(rccar2.np.argmin(abs(sum_x)),sum_x.shape)
Yi, Yj = rccar2.np.unravel_index(rccar2.np.argmin(abs(sum_y)),sum_y.shape)
Vi, Vj = rccar2.np.unravel_index(rccar2.np.argmin(abs(sum_v)),sum_v.shape)
C_alpha_erro_x_min = C_alpha_vector[Xi] # Try to catch the minimal error - varying C_s
C_alpha_erro_y_min = C_alpha_vector[Yi]
C_alpha_erro_v_min = C_alpha_vector[Vi]
C_s_erro_x_min = C_s_vector[Xj] # Try to catch the minimal error - varying C_s
C_s_erro_y_min = C_s_vector[Yj]
C_s_erro_v_min = C_s_vector[Vj]

print(sum_v)
# C_alpha_erro_x_min = C_alpha_vector[min_x_i]
# C_alpha_erro_y_min = C_alpha_vector[min_y_i]
# C_alpha_erro_v_min = C_alpha_vector[min_v_i]

print("Optimizated C_alpha for x_diff: ",C_alpha_erro_x_min)
print("Optimizated C_alpha for y_diff: ",C_alpha_erro_y_min)
print("Optimizated C_alpha for v_diff: ",C_alpha_erro_v_min)
print("Optimizated C_s for x_diff: ",C_s_erro_x_min)
print("Optimizated C_s for y_diff: ",C_s_erro_y_min)
print("Optimizated C_s for v_diff: ",C_s_erro_v_min)

# PLOTS
# rccar2.ComparisonPlot(treal, xreal, yreal, vreal, tsim, Xe, Ye, xpsimu, ypsimu)





