from math import cos, sin, tan

from libs import normalise_angle


class KinematicBicycleModel:
    def __init__(self, wheelbase: float, max_steer: float, delta_time: float=0.05): #objeto

        self.delta_time = delta_time
        self.wheelbase = wheelbase
        self.max_steer = max_steer


    def update(self, x: float, y: float, yaw: float, velocity: float, acceleration: float, steering_angle: float) -> tuple[float, ...]:
        
        new_velocity = velocity + self.delta_time * acceleration

        # Limit steering angle to physical vehicle limits       
        steering_angle = -self.max_steer if steering_angle < -self.max_steer else self.max_steer if steering_angle > self.max_steer else steering_angle

        # Compute the angular velocity
        angular_velocity = new_velocity*tan(steering_angle) / self.wheelbase

        # Compute the final state using the discrete time model
        new_x   = x + velocity*cos(yaw)*self.delta_time
        new_y   = y + velocity*sin(yaw)*self.delta_time
        new_yaw = normalise_angle(yaw + angular_velocity*self.delta_time)
        
        return new_x, new_y, new_yaw, new_velocity, steering_angle, angular_velocity