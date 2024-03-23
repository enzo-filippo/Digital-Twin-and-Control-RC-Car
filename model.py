import numpy as np
from scipy.integrate import odeint
from pylab import show, figure, plot, xlabel, ylabel, grid, legend, title, savefig, axis
from matplotlib.font_manager import FontProperties
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

        self.x = val_0[0]
        self.y = val_0[1]
        self.psi = val_0[2]
        self.x_p = val_0[3]
        self.y_p = val_0[4]
        self.psi_p = val_0[5]
        self.x_pp = val_0[6]
        self.y_pp = val_0[7]
        self.psi_pp = val_0[8]
        self.Xe_p = val_0[9]
        self.Ye_p = val_0[10]
        self.delta_p = val_0[11]

        self.f_w = wheel(self.lf, 0.0, self.mi, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, self.throttle2omega)
        self.r_w = wheel(0.0, self.lr, self.mi, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, self.throttle2omega)

        if not os.path.exists('results'):
            os.makedirs('results')
        if not os.path.exists(os.path.join('results',self.name)):
            os.makedirs(os.path.join('results',self.name))

    def run(self, t, ode_p):
        abserr, relerr, _, _ = ode_p

        coef = [self.f_w, self.r_w, self.m, self.Iz]
        val_0 = [self.x_p, self.x_pp, self.y_p, self.y_pp, self.psi_p, self.psi_pp, self.Xe_p, self.Ye_p]

        # Call the ODE solver.
        wsol = odeint(vectorfield, val_0, t, args=(coef,),atol=abserr, rtol=relerr)
        
        self.f_w.Xe = wsol[:,6] + self.f_w.lf*np.cos(wsol[:,4])
        self.f_w.Ye = wsol[:,7] + self.f_w.lf*np.sin(wsol[:,4])
        self.r_w.Xe = wsol[:,6] - self.r_w.lr*np.cos(wsol[:,4])
        self.r_w.Ye = wsol[:,7] - self.r_w.lr*np.sin(wsol[:,4])


        with open(os.path.join('results',self.name,'sim.dat'), 'w') as f:
            for t1, w1, xef, yef, xer, yer in zip(t, wsol, self.f_w.Xe, self.f_w.Ye, self.r_w.Xe, self.r_w.Ye):
                print(t1, w1[0], w1[1], w1[2], w1[3], w1[4], w1[5], w1[6], w1[7], xef, yef, xer, yer, file=f)

    def plot(self):
        t, x, xp, y, yp, psi, psip, Xe, Ye, Xef, Yef, Xer, Yer = np.loadtxt(os.path.join('results',self.name,'sim.dat'), unpack=True)

        figure(1, figsize=(6, 4.5))
        xlabel('t')
        ylabel('x')
        grid(True)
        lw = 1
        # axis('equal')
        plot(t, x, 'b', linewidth=lw)
        savefig(os.path.join('results',self.name,'simx.pdf'), dpi=100)
        
        figure(2, figsize=(6, 4.5))
        xlabel('t')
        ylabel('y')
        grid(True)
        lw = 1
        # axis('equal')
        plot(t, y, 'r', linewidth=lw)
        savefig(os.path.join('results',self.name,'simy.pdf'), dpi=100)
        
        figure(3, figsize=(6, 4.5))
        xlabel('Xe')
        ylabel('Ye')
        grid(True)
        lw = 1
        axis('equal')
        plot(Xe, Ye, 'k', linewidth=lw)
        savefig(os.path.join('results',self.name,'sim.pdf'), dpi=100)

        figure(4, figsize=(6, 4.5))
        xlabel('t')
        ylabel('psi')
        grid(True)
        lw = 1
        axis('equal')
        plot(t, psi, 'k', linewidth=lw)
        savefig(os.path.join('results',self.name,'psi.pdf'), dpi=100)

        figure(5, figsize=(6, 4.5))
        xlabel('t')
        ylabel('delta')
        grid(True)
        lw = 1
        plot(t, self.f_w.delta(t), 'k', linewidth=lw)
        savefig(os.path.join('results',self.name,'delta.pdf'), dpi=100)
        
        figure(6, figsize=(6, 4.5))
        xlabel('t')
        ylabel('throttle')
        grid(True)
        lw = 1
        plot(t, self.f_w.throttle(t), 'k', linewidth=lw)
        savefig(os.path.join('results',self.name,'throttle.pdf'), dpi=100)

        show()

class wheel():
    def __init__(self, lf, lr, Lw, r, mi, C_s, C_alpha, Fz, throttle2omega):
        self.lf = lf
        self.lr = lr
        self.Lw = Lw
        self.r = r
        self.mi = mi
        self.C_s = C_s
        self.C_alpha = C_alpha
        self.Fz = Fz
        self.throttle2omega = throttle2omega
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
        valor = t*0 + 255
        return valor

    def delta(self,t):
        angulo = 20
        multiplicador = 1/360
        t1 = 2.5
        x1 = t - t1
        t2 = 5
        x2 = t - t2

        valor_rad = np.heaviside(x1,1) - np.heaviside(x2,1)
        valor_deg = multiplicador*angulo*valor_rad
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
    