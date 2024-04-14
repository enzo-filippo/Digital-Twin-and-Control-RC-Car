import model
import real_data

# ODE Solver parameters
abserr = 1.0e-8
relerr = 1.0e-6
starttime = 5
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
x, y = voiture.run(t, ode_param)
voiture.plot()
# voiture.anim(step)

treal, xreal, yreal, vreal, areal = real_data.run(throttlereal)
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



