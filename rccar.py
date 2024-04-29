import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os

# non-linear lateral bicycle model
class NonLinearBycicle():
    def __init__(self, name, parameters, val_0):
        self.name = name
        self.max_steer = np.radians(parameters[0])   
        self.m = parameters[1]             
        self.Iz = parameters[2]            
        self.lf = parameters[3]
        self.lr = parameters[4]
        self.Lw = parameters[5]
        self.r = parameters[6]
        self.mi = parameters[7]
        self.C_s = parameters[8]
        self.C_alpha = parameters[9]
        self.Fz = parameters[10]
        self.throttle2omega = parameters[11]
        self.throttle_parameters = parameters[12]
        self.delta_parameters = parameters[13]

        self.x = val_0[0]
        self.y = val_0[1]
        self.psi = val_0[2]
        self.x_p = val_0[3]
        self.y_p = val_0[4]
        self.psi_p = val_0[5]
        self.x_pp = val_0[6]
        self.y_pp = val_0[7]
        self.psi_pp = val_0[8]
        self.Xe = val_0[9]
        self.Ye = val_0[10]
        self.delta_p = val_0[11]

        self.f_w = wheel(self.lf, 0.0, self.mi, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, 
                         self.throttle2omega, self.throttle_parameters, self.delta_parameters, self.max_steer)
        self.r_w = wheel(0.0, self.lr, self.mi, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, 
                         self.throttle2omega, self.throttle_parameters, self.delta_parameters, self.max_steer)

        if not os.path.exists('results'):
            os.makedirs('results')
        if not os.path.exists(os.path.join('results',self.name)):
            os.makedirs(os.path.join('results',self.name))

    def run(self, t, ode_p):
        abserr, relerr, _, _ = ode_p
        coef = [self.f_w, self.r_w, self.m, self.Iz]
        val_0 = [self.x, self.x_p, self.y, self.y_p, self.psi, self.psi_p, self.Xe, self.Ye]

        # Call the ODE solver.
        wsol = odeint(vectorfield, val_0, t, args=(coef,),atol=abserr, rtol=relerr)
        
        self.f_w.Xe = wsol[:,6] + self.f_w.lf*np.cos(wsol[:,4])
        self.f_w.Ye = wsol[:,7] + self.f_w.lf*np.sin(wsol[:,4])
        self.r_w.Xe = wsol[:,6] - self.r_w.lr*np.cos(wsol[:,4])
        self.r_w.Ye = wsol[:,7] - self.r_w.lr*np.sin(wsol[:,4])
        
        x = wsol[0] 
        xp = wsol[1] 
        y = wsol[2] 
        yp = wsol[3] 
        psi = wsol[4] 
        psip = wsol[5]
        Xe = wsol[6] 
        Ye = wsol[7] 
        Xef = self.f_w.Xe 
        Yef = self.f_w.Ye 
        Xer = self.r_w.Xe 
        Yer = self.r_w.Ye

        with open(os.path.join('results',self.name,'sim.dat'), 'w') as f:
            for t1, w1, xef, yef, xer, yer in zip(t, wsol, self.f_w.Xe, self.f_w.Ye, self.r_w.Xe, self.r_w.Ye):
                print(t1, w1[0], w1[1], w1[2], w1[3], w1[4], w1[5], w1[6], w1[7], xef, yef, xer, yer, file=f)

class wheel():
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
        if self.lf == 0:
            self.name = "r_w"
        if self.lr == 0:
            self.name = "f_w"

    def update_dugoff_forces(self, throttle, delta, psi_p, x_p, y_p):
        omega = self.throttle2omega*throttle
        Vx = x_p
        Vy = y_p
        
        if throttle > 0:
            s = (self.r*omega - Vx)/(self.r*omega)
        if throttle <= 0:
            s = (self.r*omega - Vx)/np.abs(Vx)

        if(self.lr != 0):
            signal = -1
        elif(self.lf != 0):
            signal = 1

        numerador = Vy + self.lf * psi_p - self.lr * psi_p
        denominador = Vx + signal*self.Lw/2 * psi_p
        
        theta_V = np.arctan(numerador/denominador) 
        if np.isnan(theta_V):
            theta_V = 0
        alpha = delta - theta_V

        C_lambda = (self.mi * self.Fz*(1+s))/(2*np.sqrt((self.C_s*s)**2 + (self.C_alpha*np.tan(alpha)**2)))
        if C_lambda < 1:
            f_C_lambda = (2-C_lambda)*C_lambda
        else:
            f_C_lambda = 1 

        Fxp = self.C_s *(s/(1+s))*f_C_lambda
        Fyp = self.C_alpha*(np.tan(alpha)/(1+s))*f_C_lambda

        self.Fx = Fxp*np.cos(delta) - Fyp*np.sin(delta)
        self.Fy = Fxp*np.sin(delta) + Fyp*np.cos(delta)

    def throttle(self,t):
        if self.throttle_type == "full":
            return t*0 + 127
        if self.throttle_type == "step":
            if t < self.t0_throttle:
                return t*0 + 0
            if t >= self.t0_throttle and t < self.tf_throttle:
                return t*0 + self.throttle_command
            if t > self.tf_throttle:
                return t*0 + 0

    def delta(self,t):
        if self.delta_type == "straight":
            return t*0 + 0
        if self.delta_type == "step":
            if t < self.t0_delta:
                return t*0 + 0
            if t >= self.t0_delta and t < self.tf_delta:
                return t*0 + self.delta_command
            if t > self.tf_delta:
                return t*0 + 0

def vectorfield(w, t, coef):
    """
    Defines the differential equations for the coupled system.

    Arguments:
        w :  vector of the state variables:
                  w = [x_p, x_pp, y_p, y_pp, psi_p, psi_pp, Xe_p, Ye_p]
        t :  counter
        p :  vector of the parameters:
                  p = [Fxf, Fxr, Fyf, Fyr, lf, lr, m, Iz]
    """
    x, xp, y, yp, psi, psip, Xe, Ye = w
    f_w, r_w, m, Iz = coef

    f_w.update_dugoff_forces(f_w.throttle(t), f_w.delta(t), psip, xp, yp)
    r_w.update_dugoff_forces(r_w.throttle(t), 0.0, psip, xp, yp)

    Fxf = f_w.Fx
    Fyf = f_w.Fy
    Fxr = r_w.Fx
    Fyr = r_w.Fy
    lf = f_w.lf
    lr = r_w.lr

    # Create f = (x',xp',y',yp',psi',psip', Xe', Ye'):
    f = [xp,
         (Fxf + Fxr)/m + psip*yp,
         yp,
         (Fyf + Fyr)/m - psip*xp,
         psip,
         (lf*Fyf - lr*Fyr)/Iz,
         xp*np.cos(psi)-yp*np.sin(psi),
         xp*np.sin(psi)+yp*np.cos(psi)]
    
    return f
    
def set_throttle(command, t0, tf, type):
    throttle_parameters = [command, t0, tf, type]
    return throttle_parameters

def set_delta(command, t0, tf, type):
    delta_parameters = [command, t0, tf, type]
    return delta_parameters

def anim(sim_file_directory, anim_fps):
        def read_file(filename):
            data = np.loadtxt(filename)
            return data[:, 0], data[:, 7], data[:, 8], data[:, 9], data[:, 10], data[:, 11], data[:, 12]

        def init():
            lines['cg'].set_data([], [])
            lines['fw'].set_data([], [])
            lines['rw'].set_data([], [])
            return lines.values()

        def animate(i):
            lines['cg'].set_data(x[i], y[i])
            lines['fw'].set_data(xef[i], yef[i])
            lines['rw'].set_data(xer[i], yer[i])
            lines['cgrastro'].set_data(x[:i], y[:i])
            lines['fwrastro'].set_data(xef[:i], yef[:i])
            lines['rwrastro'].set_data(xer[:i], yer[:i])
            time_legend.set_text('Time: {:.2f}'.format(t[i]))
            return (*lines.values(), time_legend)


        t, x, y, xef, yef, xer, yer = read_file(os.path.join('results',sim_file_directory,'sim.dat'))

        fig, ax = plt.subplots()
        ax.set_xlim(np.min(np.concatenate((x,y)))-0.1, np.max(np.concatenate((x,y)))+0.1)  # Adjust x-axis limits as needed
        ax.set_ylim(np.min(np.concatenate((x,y)))-0.1, np.max(np.concatenate((x,y)))+0.1)  # Adjust y-axis limits as needed
        ax.set_aspect('equal')
        lines = {'cg': ax.plot([], [], 'bo', color='blue', label='CG')[0],
                'fw': ax.plot([], [], 'ro', color='red', label='FW')[0],
                'rw': ax.plot([], [], 'go', color='green', label='RW')[0],
                'cgrastro': ax.plot([], [], 'b-', color='blue', label='CG')[0],
                'fwrastro': ax.plot([], [], 'r:', color='red')[0],
                'rwrastro': ax.plot([], [], 'g:', color='green')[0]}
        time_legend = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top', ha='left')

        # Create the animation
        ani = FuncAnimation(fig, animate, frames=len(t), init_func=init, blit=True, interval = anim_fps)
        ax.legend()

        # ani.save(os.path.join('results','curva_t_255_d_20','anim.mp4'), fps=30, extra_args=['-vcodec', 'libx264'])  
        plt.show()

def read_sim_file(sim_file_directory):
    t, x, xp, y, yp, psi, psip, Xe, Ye, Xef, Yef, Xer, Yer = np.loadtxt(os.path.join('results',sim_file_directory,'sim.dat'), unpack=True)
    return t, x, xp, y, yp, psi, psip, Xe, Ye, Xef, Yef, Xer, Yer

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
    pos = np.array(data['positions_barycentre_corr'])
    x = pos[:int((np.floor(len(pos)/2)))]/100
    y = pos[int((np.floor(len(pos)/2))):]/100
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
    print("For the Y axis we have: y0 = ", yreal[0], " and yf = ", yreal[-1])
    print("For the X axis we have: x0 = ", xreal[0], " and xf = ", xreal[-1])
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

def ComparisonPlot(treal, xreal, yreal, vreal, tsimu, xsimu, ysimu, xpsimu, ypsimu, exp_name_file):
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
    plt.plot(treal, ysimu, 'b:', label ="simulation")
    plt.legend()
    plt.savefig('figures/'+name_figures+'_Comparison_Y_axis.pdf')

    plt.figure(figsize=(6, 4.5))
    plt.xlabel("t [s]")
    plt.ylabel("error [m]")
    plt.grid(True)
    plt.axis('equal')
    plt.title(" Postion error (between simu and real) ")
    plt.plot(treal, x_dif,'r:', label ="error x")
    plt.plot(treal, y_dif, 'b:', label ="error y")
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
    plt.plot(treal[2:], v_dif, 'b:', label ="erreur de vitesse")
    plt.legend()
    plt.savefig('figures/'+name_figures+'_Error_Speed.pdf')
    plt.show()