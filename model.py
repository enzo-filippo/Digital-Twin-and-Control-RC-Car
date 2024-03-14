import numpy as np
from scipy.integrate import odeint
from pylab import figure, plot, xlabel, ylabel, grid, legend, title, savefig, axis
from matplotlib.font_manager import FontProperties

counter = 0

# non-linear lateral bicycle model
class NonLinearBycicle():
    def __init__(self, parameters, val_0):
        self.max_steer = np.radians(parameters[0])   
        self.m = parameters[1]             
        self.Iz = parameters[2]            
        self.lf = parameters[3]
        self.lr = parameters[4]
        self.Lw = parameters[5]
        self.r = parameters[6]
        self.mi = parameters[7]
        self.C_tau = parameters[8]
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
        self.delta_p = val_0[9]
        self.throttle = val_0[10]
        
        self.delta_old = self.delta_p

        self.f_w = wheel(self.lf, 0.0, self.mi, self.r, self.mi, self.C_tau, self.C_alpha, self.Fz, self.throttle2omega)
        self.r_w = wheel(0.0, self.lr, self.mi, self.r, self.mi, self.C_tau, self.C_alpha, self.Fz, self.throttle2omega)
        self.f_w.update(self.throttle[0], self.delta_p, self.psi_p)
        self.r_w.update(self.throttle[0], 0.0, self.psi_p)    

    def run(self, throttle, delta, t, ode_p):
        counter = 0.0
        abserr, relerr, _, _ = ode_p

        delta = np.clip(delta, -self.max_steer, self.max_steer)
  
        #self.x_pp = (self.f_w.Fx + self.r_w.Fx)/self.m + self.psi_p*self.y_p
        #self.y_pp = (self.f_w.Fy + self.r_w.Fy)/self.m + self.psi_p*self.x_p
        #self.psi_pp = (self.lf*self.f_w.Fy - self.lr*self.r_w.Fy)/self.Iz
        #self.delta_p = self.delta_old - delta
        #self.delta_old = delta

        coef = [self.f_w, self.r_w, self.m, self.Iz, throttle, delta]
        val_0 = [self.x_p, self.x_pp, self.y_p, self.y_pp, self.psi_p, self.psi_pp]

        # Call the ODE solver.
        wsol = odeint(vectorfield, val_0, t, args=(coef,),
                    atol=abserr, rtol=relerr)

        with open('sim.dat', 'w') as f:
            # Print & save the solution.
            for t1, w1 in zip(t, wsol):
                print(t1, w1[0], w1[1], w1[2], w1[3], w1[4], w1[5], file=f)

    def plot(self):
        t, x, xp, y, yp, psi, psip= np.loadtxt('sim.dat', unpack=True)

        figure(1, figsize=(6, 4.5))

        xlabel('x')
        ylabel('y')
        grid(True)
        lw = 1
        axis('equal')

        plot(x, y, 'b', linewidth=lw)
        savefig('sim.png', dpi=100)

class wheel():
    def __init__(self, lf, lr, Lw, r, mi, C_tau, C_alpha, Fz, throttle2omega):
        self.lf = lf
        self.lr = lr
        self.Lw = Lw
        self.r = r
        self.mi = mi
        self.C_tau = C_tau
        self.C_alpha = C_alpha
        self.Fz = Fz
        self.throttle2omega = throttle2omega

    def update(self, throttle, delta, psi_p):
        omega = self.throttle2omega*throttle
        Vx = omega*self.r
        Vy = omega*self.r

        if throttle > 0:
            tau = (self.r*omega - Vx)/(self.r*Vx)
        if throttle <= 0:
            tau = self.r*omega - Vx/np.abs(Vx)

        if(self.lr != 0):
            signal = -1
        if(self.lf != 0):
            signal = 1

        numerador = Vy + self.lf * psi_p - self.lr * psi_p
        denominador = Vx + signal*self.Lw/2 * psi_p

        self.theta_V = np.arctan(numerador/denominador)
        alpha = delta - self.theta_V

        C_lambda = (self.mi * self.Fz*(1+tau))/(2*np.sqrt(self.C_tau*tau)**2 + self.C_alpha*np.tan(alpha)**2) 
        if C_lambda < 1:
            f_C_lambda = (2-C_lambda)*C_lambda
        else:
            f_C_lambda = 1 

        Fxp = self.C_tau *(tau/(1-tau))*f_C_lambda
        Fyp = self.C_alpha*(np.tan(alpha)/(1-tau))*f_C_lambda

        self.Fx = Fxp*np.cos(delta) - Fyp*np.sin(delta)
        self.Fy = Fxp*np.sin(delta) + Fyp*np.cos(delta)

def vectorfield(w, t, coef):
    """
    Defines the differential equations for the coupled system.

    Arguments:
        w :  vector of the state variables:
                  w = [x_p, x_pp, y_p, y_pp, psi_p, psi_pp]
        t :  counter
        p :  vector of the parameters:
                  p = [Fxf, Fxr, Fyf, Fyr, lf, lr, m, Iz]
    """
    x, xp, y, yp, psi, psip = w
    f_w, r_w, m, Iz, throttle, delta = coef

    global counter 
    counter = counter + 1

    f_w.update(throttle[counter], delta[counter], psip)
    r_w.update(throttle[counter], 0.0, psip)    

    Fxf = f_w.Fx
    Fyf = f_w.Fy
    Fxr = r_w.Fx
    Fyr = r_w.Fy
    lf = f_w.lf
    lr = r_w.lr

    # Create f = (x',xp',y',yp',psi',psip'):
    f = [xp,
         (Fxf + Fxr)/m + psip*yp,
         yp,
         (Fyf + Fyr)/m + psip*xp,
         psip,
         (lf*Fyf - lr*Fyr)*Iz]
    return f
    