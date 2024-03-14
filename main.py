import model

# Parameter values
max_steer_angle = 30.0
m = 0.04
Iz = 0.000266
lf = 0.047
lr = 0.05
Lw = 0.0
r = 0.024
mi = 0.57
C_tau = 0.2
C_alpha = 0.2
Fz = m*9.98/4
min_v = 0.1
throttle2omega = 1

# ODE Solver parameters
abserr = 1.0e-8
relerr = 1.0e-6
stoptime = 5.0
numpoints = 250

# Initial conditions
# x1 and x2 are the initial displacements; y1 and y2 are the initial velocities
x0 = 0.0
y0 = 0.0
psi0 = 0.0
xp0 = 0.0
xpp0 = 0.0
yp0 = 0.0
ypp0 = 0.0
psip0 = 0.0
psipp0 = 0.0
var_delta = 0.0
throttle = 0.0

# Create the time samples for the output of the ODE solver.
# I use a large number of points, only because I want to make
# a plot of the solution that looks nice.
t = [stoptime * float(i) / (numpoints - 1) for i in range(numpoints)]
throttle = model.np.ones(len(t))
delta = model.np.zeros(len(t))

# Pack up the parameters and initial conditions:
p = [max_steer_angle, m, Iz, lf, lr, Lw, r, mi, C_tau, C_alpha, Fz, min_v, throttle2omega]
val_0 = [x0, y0, psi0, xp0, xpp0, yp0, ypp0, psip0, psipp0, var_delta, throttle]
ode_p = [abserr, relerr, stoptime, numpoints]
voiture = model.NonLinearBycicle(p, val_0)
voiture.run(throttle, delta, t, ode_p)
voiture.plot()