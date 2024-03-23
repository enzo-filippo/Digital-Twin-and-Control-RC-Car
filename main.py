import model

# ODE Solver parameters
abserr = 1.0e-8
relerr = 1.0e-6
stoptime = 20
numpoints = 1000
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
mi = 0.57 
C_s = 0.9
C_alpha = 0.9
Fz = m*9.98/4
min_v = 0.1
throttle2omega = (1/255)*10
param = [max_steer_angle, m, Iz, lf, lr, Lw, r, mi, C_s, C_alpha, Fz, min_v, throttle2omega]

# Initial conditions
x0 = 0.0
y0 = 0.0
psi0 = 0.0
xp0 = 0.0001 #must not be zero
xpp0 = 0.0
yp0 = 0.0
ypp0 = 0.0
psip0 = 0.0
psipp0 = 0.0
Xep0 = 0.0
Yep0 = 0.0
var_delta = 0.0
val_0 = [x0, y0, psi0, xp0, xpp0, yp0, ypp0, psip0, psipp0, Xep0, Yep0, var_delta]

voiture = model.NonLinearBycicle("curva_t_255_d_20", param, val_0)
voiture.run(t, ode_param)
voiture.plot()