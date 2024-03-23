import numpy as np
from scipy.integrate import odeint
from pylab import show, figure, plot, xlabel, ylabel, grid, legend, title, savefig, axis
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
        self.C_s = parameters[8]
        self.C_alpha = parameters[9]
        self.Fz = parameters[10]
        self.min_v = parameters[11]
        self.throttle2omega = parameters[12]
        self.alpha_0 = parameters[13]
        self.throttle_0 = parameters[14]

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
        
        #print(self.x_p)
        
        self.delta_old = self.delta_p

        self.f_w = wheel(self.lf, 0.0, self.mi, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, self.throttle2omega)
        self.r_w = wheel(0.0, self.lr, self.mi, self.r, self.mi, self.C_s, self.C_alpha, self.Fz, self.throttle2omega)
        # self.f_w.update(self.throttle_0, self.delta_p, self.psi_p, self.x_p, self.y_p, True, self.alpha_0)
        # self.r_w.update(self.throttle_0, 0.0, self.psi_p, self.x_p, self.y_p, True, self.alpha_0)

    def run(self, t, ode_p):
        global counter
        counter = 0
        abserr, relerr, _, _ = ode_p
  
        #self.x_pp = (self.f_w.Fx + self.r_w.Fx)/self.m + self.psi_p*self.y_p
        #self.y_pp = (self.f_w.Fy + self.r_w.Fy)/self.m + self.psi_p*self.x_p
        #self.psi_pp = (self.lf*self.f_w.Fy - self.lr*self.r_w.Fy)/self.Iz
        #self.delta_p = self.delta_old - delta
        #self.delta_old = delta

        coef = [self.f_w, self.r_w, self.m, self.Iz]
        val_0 = [self.x_p, self.x_pp, self.y_p, self.y_pp, self.psi_p, self.psi_pp, self.Xe_p, self.Ye_p]

        # Call the ODE solver.
        wsol = odeint(vectorfield, val_0, t, args=(coef,),
                    atol=abserr, rtol=relerr)

        with open('sim.dat', 'w') as f:
            # #Print & save the solution.
            for t1, w1 in zip(t, wsol):
                print(t1, w1[0], w1[1], w1[2], w1[3], w1[4], w1[5], w1[6], w1[7], file=f)

    def plot(self):
        t, x, xp, y, yp, psi, psip, Xe, Ye = np.loadtxt('sim.dat', unpack=True)

        figure(1, figsize=(6, 4.5))
        xlabel('t')
        ylabel('x')
        grid(True)
        lw = 1
        # axis('equal')
        plot(t, x, 'b', linewidth=lw)
        savefig('simx.png', dpi=100)
        
        figure(2, figsize=(6, 4.5))
        xlabel('t')
        ylabel('y')
        grid(True)
        lw = 1
        # axis('equal')
        plot(t, y, 'r', linewidth=lw)
        savefig('simy.png', dpi=100)
        
        figure(3, figsize=(6, 4.5))
        xlabel('Xe')
        ylabel('Ye')
        grid(True)
        lw = 1
        axis('equal')
        plot(Xe, Ye, 'k', linewidth=lw)
        savefig('sim.png', dpi=100)

        figure(4, figsize=(6, 4.5))
        xlabel('t')
        ylabel('delta')
        grid(True)
        lw = 1
        axis('equal')
        plot(t, delta(t), 'k', linewidth=lw)
        savefig('delta.png', dpi=100)
        
        figure(5, figsize=(6, 4.5))
        xlabel('t')
        ylabel('throttle')
        grid(True)
        lw = 1
        axis('equal')
        plot(t, throttle(t), 'k', linewidth=lw)
        savefig('throttle.png', dpi=100)

        figure(6, figsize=(6, 4.5))
        xlabel('t')
        ylabel('psi')
        grid(True)
        lw = 1
        axis('equal')
        plot(t, psi, 'k', linewidth=lw)
        savefig('psi.png', dpi=100)


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

    def update(self, throttle, delta, psi_p, x_p, y_p, initialization, alpha_0):
        #print(psi_p, x_p, y_p)
        omega = self.throttle2omega*throttle
        #print(self.name,"omega:", omega)
        #print(self.name,"throttle:", throttle)
        Vx = x_p
        Vy = y_p
        #print(self.name,"Vx:",Vx)
        #print(self.name,"Vy:",Vy)
        
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
        #print(self.name,"num:",numerador)
        #print(self.name,"den:",denominador)
        
        theta_V = np.arctan(numerador/denominador) 
        if np.isnan(theta_V):
            theta_V = 0
        #print(self.name,"theta_V:",theta_V)
        alpha = delta - theta_V

        #print(self.name,"alpha:",alpha)

        C_lambda = (self.mi * self.Fz*(1+s))/(2*np.sqrt((self.C_s*s)**2 + (self.C_alpha*np.tan(alpha)**2)))
        if C_lambda < 1:
            f_C_lambda = (2-C_lambda)*C_lambda
        else:
            f_C_lambda = 1 
        
        #print(self.name,"C_lambda:",C_lambda)
        #print(self.name,"s:",s)

        Fxp = self.C_s *(s/(1+s))*f_C_lambda
        Fyp = self.C_alpha*(np.tan(alpha)/(1+s))*f_C_lambda

        self.Fx = Fxp*np.cos(delta) - Fyp*np.sin(delta)
        self.Fy = Fxp*np.sin(delta) + Fyp*np.cos(delta)
        #print(self.name,"Fx, Fy: ", self.Fx, self.Fy)

def throttle(t):
    return t - t + 255

def delta(t):
    angulo = 20
    multiplicador = 1/360
    t1 = 2.5
    x1 = t - t1
    t2 = 5
    x2 = t - t2

    # valor = (np.sin(2*np.pi*(1/10)*(t-t0)) +1)
    valor_rad = np.heaviside(x1,1) - np.heaviside(x2,1)
    valor_deg = multiplicador*angulo*valor_rad
    return valor_deg

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

    global counter 
    global delta
    global throttle
    f_w.update(throttle(t), delta(t), psip, xp, yp, False, 0.0)
    r_w.update(throttle(t), 0.0, psip, xp, yp, False, 0.0)
 
    counter = counter + 1
    #print("counter:", counter)
    #print("xp =",xp)

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
    
    #print(f)
    
    return f
    