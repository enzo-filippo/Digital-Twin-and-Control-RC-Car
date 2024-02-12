## Bycicle model - consider only two wheels 
import numpy as np
from math import cos, sin, tan
from libs import normalise_angle

class BycicleKinematicModel:
    def __init__(self, wheelRadius: float, max_steer: float, delta_time: float):
        self.delta_time = delta_time
        self.wheelRadius = wheelRadius
        self.max_steer = max_steer
    
    def __update__(self, x : float, y : float, yaw: float, velocity: float, acceleration: float, steering_angle: float):

        if steering_angle > self.max_steer:
            steering_angle = self.max_steer
        elif steering_angle < -self.max_steer
            steering_angle = - self.max_steer
        else
            steering_angle = steering_angle

        n_velocity = acceleration*self.delta_time + self.velocity

        angular_velocity = n_velocity*tan(steering_angle)/self.wheelRadius

        n_x = x + velocity*cos(yaw)*self.delta_time
        n_y = y + velocity*sin(yaw)*self.delta_time
        n_yaw = yaw + angular_velocity
    return steering_angle, n_velocity, angular_velocity, n_x, n_y, n_yaw

def DriftRate(r_eff, v_x, a_x, omega):
    if a_x > 0: ## glissement positif
        tau_x = (r_eff * omega - v_x)/(r_eff * omega)
    elif a_x < 0: ## glissement negatif
        tau_x = (r_eff * omega - v_x)/(abs(v_x))
    else
        tau_x = 0

'''
    C_tau - Coefficient de rigidite longitudinal du pneu
    C_alpha - Coefficient de rigidite de derive du pneu
'''
def WheelForces(C_tau, tau_x, C_alpha, alpha):
    F_x = C_tau * tau_x
    F_y = C_alpha * alpha


## modelo de dugoff

def CoefDugoff(C_tau, C_alpha, alpha, tau_x, mu, F_z):
    lambda_i = (mu*F_z*(1+tau_x))/(2*np.sqrt((C_tau*tau_x)**)+(C_alpha*np.tan(alpha)**))
