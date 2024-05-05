import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import os
from scipy.interpolate import interp1d
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

glissement_f = np.array([])
glissement_r = np.array([])

class NonLinearBicycle():
    def __init__(self, name, parameters, val_0):
        self.name = name
        self.parameters = parameters
        self.val_0 = val_0
        self.setup_parameters()

    def setup_parameters(self):
        params = self.parameters
        self.max_steer = np.radians(params[0])   
        self.m = params[1]             
        self.Iz = params[2]            
        self.lf = params[3]
        self.lr = params[4]
        self.Lw = params[5]
        self.r = params[6]
        self.mi = params[7]
        self.C_s = params[8]
        self.C_alpha = params[9]
        self.Fz = params[10]
        self.throttle2omega = params[11]
        self.throttle_parameters = params[12]
        self.delta_parameters = params[13]

        self.f_w = Wheel(self.lf, 0.0, self.Lw, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, self.throttle2omega, self.throttle_parameters, self.delta_parameters, self.max_steer)
        self.r_w = Wheel(0.0, self.lr, self.Lw, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, self.throttle2omega, self.throttle_parameters, self.delta_parameters, self.max_steer)

        if not os.path.exists('results'):
            os.makedirs('results')
        if not os.path.exists(os.path.join('results', self.name)):
            os.makedirs(os.path.join('results', self.name))

    def run(self, t, ode_p):
        abserr, relerr = ode_p[:2]
        t_span = (t[0], t[-1])

        sol = solve_ivp(lambda t, w: vectorfield(t, w, self),
                        t_span, self.val_0, t_eval=t, atol=abserr, rtol=relerr, method='RK45')

        if not sol.success:
            raise RuntimeError("ODE solver failed to find a solution")

        wsol = sol.y.T

        x = wsol[0] 
        xp = wsol[1] 
        y = wsol[2] 
        yp = wsol[3] 
        psi = wsol[4] 
        psip = wsol[5]
        Xe = wsol[6] 
        Ye = wsol[7] 
        Xef1 = self.f_w.Xe + self.f_w.lf
        Yef1 = self.f_w.Ye + self.f_w.Lw/2
        Xer1 = self.r_w.Xe - self.r_w.lr
        Yer1 = self.r_w.Ye + self.f_w.Lw/2
        Xef2 = self.f_w.Xe + self.f_w.lf
        Yef2 = self.f_w.Ye - self.f_w.Lw/2
        Xer2 = self.r_w.Xe - self.r_w.lr
        Yer2 = self.r_w.Ye - self.f_w.Lw/2

        throttle_values = np.zeros(len(t))
        delta_values = np.zeros(len(t))
        for i in range(len(t)):
            throttle_values[i] = self.f_w.throttle(t[i])
            delta_values[i] = self.f_w.delta(t[i])

        Xef1, Yef1, Xer1, Yer1, Xef2, Yef2, Xer2, Yer2 = [], [], [], [], [], [], [], []

        for state in wsol:
            Xef1.append(self.f_w.Xe + self.f_w.lf)
            Yef1.append(self.f_w.Ye + self.f_w.Lw / 2)
            Xer1.append(self.r_w.Xe - self.r_w.lr)
            Yer1.append(self.r_w.Ye + self.f_w.Lw / 2)
            Xef2.append(self.f_w.Xe + self.f_w.lf)
            Yef2.append(self.f_w.Ye - self.f_w.Lw / 2)
            Xer2.append(self.r_w.Xe - self.r_w.lr)
            Yer2.append(self.r_w.Ye - self.f_w.Lw / 2)

        global glissement_r, glissement_f
        dense_t = np.linspace(0, t[-1], len(glissement_f))
        interp_function_f = interp1d(dense_t, glissement_f, kind='linear')
        interp_function_r = interp1d(dense_t, glissement_r, kind='linear')
        resampled_glissement_f = interp_function_f(t)
        resampled_glissement_r = interp_function_r(t)

        with open(os.path.join('results',self.name,'sim.dat'), 'w') as f:
            for t1, w1, xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r in zip(t, wsol, Xef1, Yef1, Xer1, Yer1, Xef2, Yef2, Xer2, Yer2, throttle_values, delta_values, resampled_glissement_f, resampled_glissement_r):
                print(t1, w1[0], w1[1], w1[2], w1[3], w1[4], w1[5], w1[6], w1[7], xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r, file=f)


class Wheel():
    def __init__(self, lf, lr, Lw, r, mi, C_s, C_alpha, Fz, throttle2omega, throttle_parameters, delta_parameters, max_steer):
        self.lf = lf
        self.lr = lr
        self.Lw = Lw
        self.r = r
        self.mi = mi
        self.C_s = C_s
        self.C_alpha = C_alpha
        self.Fz = Fz
        self.throttle2omega = throttle2omega
        self.throttle_command, self.t0_throttle, self.tf_throttle, self.throttle_type = throttle_parameters
        self.delta_command, self.t0_delta, self.tf_delta, self.delta_type = delta_parameters
        self.max_steer = max_steer
        self.Xe = 0
        self.Ye = 0

    def update_dugoff_forces(self, throttle, delta, psi_p, x_p, y_p):
        omega = self.throttle2omega * throttle
        Vx = x_p
        Vy = y_p
        omega_r = self.r * omega

        if np.round(omega_r, 2) > np.round(Vx, 2):
            self.s = (omega_r - Vx) / (omega_r)
        else:
            self.s = (omega_r - Vx) / np.abs(Vx)

        if self.s == -1 or np.isneginf(self.s):
            self.s = -0.99
        if np.isnan(self.s) or np.isposinf(self.s):
            self.s = 0.99

        
        # print("Vx:", Vx, "s:", self.s)
        numerador = Vy + self.lf * psi_p - self.lr * psi_p
        denominador = Vx

        theta_V = np.arctan(numerador / denominador) if denominador != 0 else 0
        alpha = delta - theta_V

        C_lambda = (self.mi * self.Fz * (1 + self.s)) / (2 * np.sqrt((self.C_s * self.s**2) + (self.C_alpha * np.tan(alpha)**2)))
        f_C_lambda = (2 - C_lambda) * C_lambda if C_lambda < 1 else 1

        Fxp = self.C_s * (self.s / (1 + self.s)) * f_C_lambda
        Fyp = self.C_alpha * (np.tan(alpha) / (1 + self.s)) * f_C_lambda

        self.Fx = Fxp * np.cos(delta) - Fyp * np.sin(delta)
        self.Fy = Fxp * np.sin(delta) + Fyp * np.cos(delta)

        return self.Fx, self.Fy, self.s

    def throttle(self, t):
        if self.throttle_type == "full":
            return 127
        elif self.throttle_type == "step":
            if t < self.t0_throttle:
                return t*0 + 0
            elif self.t0_throttle <= t < self.tf_throttle:
                return t*0 + self.throttle_command*(1/(1+np.exp(-1.5*t)))
            else:
                return t*0 + 0

    def delta(self, t):
        if self.delta_type == "straight":
            return t*0
        elif self.delta_type == "step":
            if t < self.t0_delta:
                return t*0 + 0
            elif self.t0_delta <= t < self.tf_delta:
                return t*0 + self.delta_command*(1/(1+np.exp(-1.5*t)))
            else:
                return t*0 + 0

def vectorfield(t, w, vehicle):
    x, xp, y, yp, psi, psip, Xe, Ye = w
    f_w, r_w, m, Iz = vehicle.f_w, vehicle.r_w, vehicle.m, vehicle.Iz

    Fxf, Fyf, s_f = f_w.update_dugoff_forces(f_w.throttle(t), f_w.delta(t), psip, xp, yp)
    Fxr, Fyr, s_r = r_w.update_dugoff_forces(r_w.throttle(t), 0.0, psip, xp, yp)

    global glissement_f, glissement_r
    glissement_f = np.append(glissement_f, s_f)
    glissement_r = np.append(glissement_r, s_r)

    lf = f_w.lf
    lr = r_w.lr

    f = [xp,
         (Fxf + Fxr) / m + psip * yp,
         yp,
         (Fyf + Fyr) / m - psip * xp,
         psip,
         (lf * Fyf - lr * Fyr) / Iz,
         xp * np.cos(psi) - yp * np.sin(psi),
         xp * np.sin(psi) + yp * np.cos(psi)]

    return f

def set_throttle(command, t0, tf, type):
    return [command, t0, tf, type]

def set_delta(command, t0, tf, type):
    return [command, t0, tf, type]

def read_sim_file(sim_file_directory):
    t, x, xp, y, yp, psi, psip, Xe, Ye,  xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r = np.loadtxt(os.path.join('results',sim_file_directory,'sim.dat'), unpack=True)
    return t, x, xp, y, yp, psi, psip, Xe, Ye,  xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r

def find_closest_value_position(arr, value):
    absolute_diff = np.abs(arr - value)
    closest_index = np.argmin(absolute_diff)
    return closest_index

def read_exp_file(exp_file_directory, file_name, initial_time):
    data = {}
    with open(os.path.join(exp_file_directory, file_name), 'r') as file:
        for line in file:
            if line.strip():  # Ignora linhas em branco
                key, values = line.strip().split(':')
                data[key.strip()] = list(map(float, values.split()))
    t = np.array(data['abscisse'])
    if 'positions_barycentre_corr' in data:
        pos = np.array(data['positions_barycentre_corr'])
        x = pos[:int((np.floor(len(pos)/2)))]/100
        y = pos[int((np.floor(len(pos)/2))):]/100
    else:
        x = np.array(data['positions_barycentre_corr_x'])/100
        y = np.array(data['positions_barycentre_corr_y'])/100
    v = np.array(data['liste_vitesse_long_corr'])/100
    a = np.array(data['acceleration_corr'])/100
    
    t_max =  np.max(t)-np.min(t)
    length = len(x)
    treal = t
    step = 0.1
    stoptime = treal[-1]-initial_time
    numpoints = int(stoptime/step)+1
    tsim = np.linspace(0,stoptime,numpoints)

    position = find_closest_value_position(treal, initial_time)
    t_initial_real = treal[-1] - tsim[-1]
    treal += -t_initial_real
    treal = treal[position:]
    xreal = x[position+1:]
    yreal = y[position+1:]
    vreal = v[position:]
    areal = a[position-1:]

    #colocar modificacoes aqui
    t0 = treal[0]
    Xe0 = xreal[0]
    Ye0 = yreal[0]
    v0 = vreal[0]
    a0 = areal[0]
    psi0_tout_droit = np.arctan((yreal[-1] - yreal[0])/(xreal[-1]-xreal[0]))
    # print("For the Y axis we have: y0 = ", yreal[0], " and yf = ", yreal[-1])
    # print("For the X axis we have: x0 = ", xreal[0], " and xf = ", xreal[-1])
    return treal, tsim, stoptime, numpoints, xreal, yreal, vreal, areal, t_max, length, t0, Xe0, Ye0, v0, a0, psi0_tout_droit

def difference(sim_position, real_position):
    size = len(sim_position)
    absolute_diff = np.zeros(size)
    sum = 0
    for i in range(size):
        absolute_diff[i] = sim_position[i] - real_position[i]
        sum += absolute_diff[i]
    return abs(absolute_diff), abs(sum)

def plot(x,y, labelx, labely):
    plt.xlabel(labelx)
    plt.ylabel(labely)
    plt.grid(True)
    # axis('equal')
    lw = 1
    plt.plot(x, y, 'b', linewidth=lw)

def ComparisonPlot(treal, xreal, yreal, vreal, tsimu, xsimu, ysimu, xpsimu, ypsimu, tv, dv, s_f, s_r, exp_name_file):
    vsimu = np.sqrt((xpsimu**2 + ypsimu**2))
    x_dif, _ = difference(xsimu,xreal)
    y_dif, _ = difference(ysimu,yreal)
    v_dif, _ = difference(vsimu[:-2],vreal[1:])

    name_figures = exp_name_file.replace(".txt","")
   
    plt.figure(figsize=(6, 4.5))
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    plt.grid(True)
    plt.axis('equal')
    plt.title(" Position (comparison of curves two axis) ")
    plt.plot(xreal, yreal,'r:', label ="réel")
    plt.plot(xsimu, ysimu, 'b:', label ="simulation")
    plt.legend()
    plt.savefig('figures/'+name_figures+'_Comparison_X_and_Y_axis.pdf')


    plt.figure(figsize=(6, 4.5))
    plt.xlabel("t [s]")
    plt.ylabel("x [m]")
    plt.grid(True)
    plt.axis('equal')
    plt.title(" Position (comparison of curves - X axis) ")
    plt.plot(treal, xreal,'r:', label ="réel")
    plt.plot(tsimu, xsimu, 'b:', label ="simulation")
    plt.legend()
    plt.savefig('figures/'+name_figures+'_Comparison_X_axis.pdf')

    plt.figure(figsize=(6, 4.5))
    plt.xlabel("t [s]")
    plt.ylabel("y [m]")
    plt.grid(True)
    plt.axis('equal')
    plt.title(" Position (comparison of curves - Y axis) ")
    plt.plot(treal, yreal,'r:', label ="réel")
    plt.plot(tsimu, ysimu, 'b:', label ="simulation")
    plt.legend()
    plt.savefig('figures/'+name_figures+'_Comparison_Y_axis.pdf')

    plt.figure(figsize=(6, 4.5))
    plt.xlabel("t [s]")
    plt.ylabel("error [m]")
    plt.grid(True)
    plt.axis('equal')
    plt.title(" Postion error (between simu and real) ")
    plt.plot(tsimu, x_dif,'r:', label ="error x")
    plt.plot(tsimu, y_dif, 'b:', label ="error y")
    plt.legend()
    plt.savefig('figures/'+name_figures+'_Error_Position_X_and_Y.pdf')

    plt.figure(figsize=(6, 4.5))
    plt.xlabel("t [s]")
    plt.ylabel("v [m/s]")
    plt.grid(True)
    plt.axis('equal')
    plt.title(" Speed (comparison of curves) ")
    plt.plot(treal, vreal,'r:', label ="réel")
    plt.plot(tsimu, vsimu, 'b:', label ="simulation")
    plt.legend()
    plt.savefig('figures/'+name_figures+'_Comparison_Speed.pdf')

    plt.figure(figsize=(6, 4.5))
    plt.xlabel("t [s]")
    plt.ylabel("error [m/s]")
    plt.grid(True)
    plt.axis('equal')
    plt.title(" Speed error (between simu and real) ")
    plt.plot(tsimu[2:], v_dif, 'b:', label ="erreur de vitesse")
    plt.legend()
    plt.savefig('figures/'+name_figures+'_Error_Speed.pdf')

    plt.figure(figsize=(6, 4.5))
    plt.xlabel("throttle")
    plt.ylabel("time [s]")
    plt.grid(True)
    plt.title(" Throttle input ")
    plt.plot(tsimu, tv,'r:')
    plt.savefig('figures/'+name_figures+'_Throttle.pdf')

    plt.figure(figsize=(6, 4.5))
    plt.xlabel("delta")
    plt.ylabel("time [s]")
    plt.grid(True)
    plt.title(" Delta input ")
    plt.plot(tsimu, dv,'r:')
    plt.savefig('figures/'+name_figures+'_Delta.pdf')

    plt.figure(figsize=(6, 4.5))
    plt.xlabel("time [s]")
    plt.ylabel("glissement [%]")
    plt.grid(True)
    plt.title(" Glissement")
    plt.plot(tsimu, s_f*100,'r:', label ="frontal")
    plt.plot(tsimu, s_r*100,'b:', label ="arriere")
    plt.legend()
    plt.savefig('figures/'+name_figures+'_glissement.pdf')

    plt.show()

def run_animation( sim_file_directory, fps=30):

    # Car parameters
    lf = 0.047  # Distance from center of mass to front wheel
    lr = 0.05   # Distance from center of mass to rear wheel
    Lw = 0.03   # Half-axle distance from the center of mass to the wheels
    car_length = lf+lr  # Estimated car length
    car_width = 2 * Lw  # Car width is twice the half-axle distance

    # Wheel dimensions
    wheel_length = 0.02  # Length of each wheel (along the direction of the car)
    wheel_width = 0.01  # Width of each wheel (perpendicular to the direction of the car)

    # Load data
    data = read_sim_file(sim_file_directory)

    # Unpack your data
    (tsimu, xsimu, xpsimu, ysimu, ypsimu, psi, psip, Xe, Ye,
    xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r) = data


    # Initialize the car body as a rectangle
    car_body = Rectangle((0, 0), car_length, car_width, angle=0, color='blue', alpha=0.7)


    # Initialize rectangles for each wheel
    wheels = [Rectangle((0, 0), wheel_length, wheel_width, color='red', alpha=0.7) for _ in range(4)]


    # Unpack data
    (tsimu, xsimu, xpsimu, ysimu, ypsimu, psi, psip, Xe, Ye,
     xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r) = data

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(np.min(Xe) - 0.5, np.max(Xe) + 0.5)
    ax.set_ylim(np.min(Ye) - 0.5, np.max(Ye) + 0.5)
    ax.set_aspect('equal')

    # Initialize the car body as a rectangle
    car_body = Rectangle((0, 0), car_length, car_width, angle=0, color='blue', alpha=0.7)
    ax.add_patch(car_body)

    # Initialize rectangles for each wheel
    wheels = [Rectangle((0, 0), wheel_length, wheel_width, color='red', alpha=0.7) for _ in range(4)]
    for wheel in wheels:
        ax.add_patch(wheel)

    # Trails for wheels
    trails = [ax.plot([], [], 'k:', linewidth=1.5)[0] for _ in range(4)]
    trail_length = 50  # Number of past positions to remember for the trail

    # Marker for the center of mass and time display
    center_of_mass, = ax.plot([], [], 'o', markersize=3)
    time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes)

    def init():
        car_body.set_xy((-0.1, -0.1))
        center_of_mass.set_data([], [])
        for trail in trails:
            trail.set_data([], [])
        time_text.set_text('')
        return [car_body, center_of_mass] + wheels + trails + [time_text]

    def update(frame):
        # Update car body and center of mass
        car_x = Xe[frame] - car_length / 2 * np.cos(psi[frame]) + car_width / 2 * np.sin(psi[frame])
        car_y = Ye[frame] - car_length / 2 * np.sin(psi[frame]) - car_width / 2 * np.cos(psi[frame])
        car_body.set_xy((car_x, car_y))
        car_body.angle = np.degrees(psi[frame])
        center_of_mass.set_data(Xe[frame], Ye[frame])
        time_text.set_text(f'Time: {tsimu[frame]:.2f}s')

        # Update wheel positions and orientations
        wheel_positions = np.array([
            [Xe[frame] + lf * np.cos(psi[frame]) - Lw * np.sin(psi[frame]),
             Ye[frame] + lf * np.sin(psi[frame]) + Lw * np.cos(psi[frame])],
            [Xe[frame] + lf * np.cos(psi[frame]) + Lw * np.sin(psi[frame]),
             Ye[frame] + lf * np.sin(psi[frame]) - Lw * np.cos(psi[frame])],
            [Xe[frame] - lr * np.cos(psi[frame]) - Lw * np.sin(psi[frame]),
             Ye[frame] - lr * np.sin(psi[frame]) + Lw * np.cos(psi[frame])],
            [Xe[frame] - lr * np.cos(psi[frame]) + Lw * np.sin(psi[frame]),
             Ye[frame] - lr * np.sin(psi[frame]) - Lw * np.cos(psi[frame])]
        ])
        for i, wheel in enumerate(wheels):
            wheel_center_x = wheel_positions[i, 0] - wheel_length / 2 * np.cos(psi[frame]) + wheel_width / 2 * np.sin(psi[frame])
            wheel_center_y = wheel_positions[i, 1] - wheel_length / 2 * np.sin(psi[frame]) - wheel_width / 2 * np.cos(psi[frame])
            wheel.set_xy((wheel_center_x, wheel_center_y))
            wheel.angle = np.degrees(psi[frame] - dv[frame]) if i < 2 else np.degrees(psi[frame])

            # Update trails
            xdata, ydata = trails[i].get_data()
            xdata = np.append(xdata, wheel_center_x)
            ydata = np.append(ydata, wheel_center_y)
            if len(xdata) > trail_length:
                xdata = xdata[-trail_length:]
                ydata = ydata[-trail_length:]
            trails[i].set_data(xdata, ydata)

        return [car_body, center_of_mass] + wheels + trails + [time_text]

    interval = 1000 / fps  # Interval in milliseconds
    ani = FuncAnimation(fig, update, frames=len(tsimu), init_func=init, blit=True, interval=interval)
    plt.show()