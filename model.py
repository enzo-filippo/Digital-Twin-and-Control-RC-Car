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
        self.min_v = parameters[11]
        self.throttle2omega = parameters[12]
        self.throttlereal = parameters[13]

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

        self.f_w = wheel(self.lf, 0.0, self.mi, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, self.throttle2omega, self.throttlereal)
        self.r_w = wheel(0.0, self.lr, self.mi, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, self.throttle2omega, self.throttlereal)

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
        


        with open(os.path.join('results',self.name,'sim.dat'), 'w') as f:
            for t1, w1, xef, yef, xer, yer in zip(t, wsol, self.f_w.Xe, self.f_w.Ye, self.r_w.Xe, self.r_w.Ye):
                print(t1, w1[0], w1[1], w1[2], w1[3], w1[4], w1[5], w1[6], w1[7], xef, yef, xer, yer, file=f)
        
        return wsol[:,6], wsol[:,7]

    def plot(self):
        t, x, xp, y, yp, psi, psip, Xe, Ye, Xef, Yef, Xer, Yer = np.loadtxt(os.path.join('results',self.name,'sim.dat'), unpack=True)

        plt.figure(figsize=(6, 4.5))
        plt.xlabel('t')
        plt.ylabel('x')
        plt.grid(True)
        lw = 1
        # axis('equal')
        plt.plot(t, x, 'b', linewidth=lw)
        plt.savefig(os.path.join('results',self.name,'simx.pdf'), dpi=100)
        
        plt.figure(figsize=(6, 4.5))
        plt.xlabel('t')
        plt.ylabel('y')
        plt.grid(True)
        lw = 1
        # axis('equal')
        plt.plot(t, y, 'r', linewidth=lw)
        plt.savefig(os.path.join('results',self.name,'simy.pdf'), dpi=100)
        
        plt.figure(figsize=(6, 4.5))
        plt.xlabel('Xe')
        plt.ylabel('Ye')
        plt.grid(True)
        lw = 1
        plt.axis('equal')
        plt.plot(Xe, Ye, 'k', linewidth=lw)
        plt.savefig(os.path.join('results',self.name,'sim.pdf'), dpi=100)

        plt.figure(figsize=(6, 4.5))
        plt.xlabel('t')
        plt.ylabel('psi')
        plt.grid(True)
        lw = 1
        plt.axis('equal')
        plt.plot(t, psi, 'k', linewidth=lw)
        plt.savefig(os.path.join('results',self.name,'psi.pdf'), dpi=100)

        plt.figure(figsize=(6, 4.5))
        plt.xlabel('t')
        plt.ylabel('delta f')
        plt.grid(True)
        lw = 1
        plt.plot(t, self.f_w.delta(t), 'k', linewidth=lw)
        plt.savefig(os.path.join('results',self.name,'delta.pdf'), dpi=100)

        plt.figure(figsize=(6, 4.5))
        plt.xlabel('t')
        plt.ylabel('delta r')
        plt.grid(True)
        lw = 1
        plt.plot(t, self.r_w.delta(t), 'k', linewidth=lw)
        plt.savefig(os.path.join('results',self.name,'delta.pdf'), dpi=100)

        plt.figure(figsize=(6, 4.5))
        plt.xlabel('t')
        plt.ylabel('throttle f')
        plt.grid(True)
        lw = 1
        plt.plot(t, self.f_w.throttle(t), 'k', linewidth=lw)
        plt.savefig(os.path.join('results',self.name,'throttle.pdf'), dpi=100)

        plt.figure(figsize=(6, 4.5))
        plt.xlabel('t')
        plt.ylabel('throttle r')
        plt.grid(True)
        lw = 1
        plt.plot(t, self.r_w.throttle(t), 'k', linewidth=lw)
        plt.savefig(os.path.join('results',self.name,'throttle.pdf'), dpi=100)


        self.velocidade = np.sqrt((xp**2 + yp**2))
        plt.figure(figsize=(6, 4.5))
        plt.xlabel('t')
        plt.ylabel('vitesse')
        plt.grid(True)
        lw = 1
        plt.plot(t, self.velocidade, 'k', linewidth=lw)
        plt.savefig(os.path.join('results',self.name,'vitesse.pdf'), dpi=100)

        plt.show()

    def anim(self, anim_fps):
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

        t, x, y, xef, yef, xer, yer = read_file(os.path.join('results',self.name,'sim.dat'))

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

        

class wheel():
    def __init__(self, lf, lr, Lw, r, mi, C_s, C_alpha, Fz, throttle2omega, throttle):
        self.lf = lf
        self.lr = lr
        self.Lw = Lw
        self.r = r
        self.mi = mi
        self.C_s = C_s
        self.C_alpha = C_alpha
        self.Fz = Fz
        self.throttle2omega = throttle2omega
        self.throttlereal = throttle
        if self.lf == 0:
            self.name = "r_w"
        if self.lr == 0:
            self.name = "f_w"

    def update(self, throttle, delta, psi_p, x_p, y_p):
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
        return t*0 + self.throttlereal

    def delta(self,t):
        # if(self.lr != 0):
            # valor_deg = 0
        # elif(self.lf != 0):
            # angulo = np.radians(10)
            # t1 = 0.5
            # x1 = t - t1
            # t2 = 4
            # x2 = t - t2

            # funcao = np.heaviside(x1,1) - np.heaviside(x2,1)
            # valor_deg = angulo*funcao
        valor_deg = 0
        return t*0 + valor_deg

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

    f_w.update(f_w.throttle(t), f_w.delta(t), psip, xp, yp)
    r_w.update(r_w.throttle(t), 0.0, psip, xp, yp)

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
    