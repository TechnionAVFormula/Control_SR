import math
import numpy as np
from sympy import solve
from sympy.abc import x

class PurePursuitController:
    """[this a class to define a controller object of a pure pursuite controller
        
        the controller gets
            the current state of the car(x,y coordinates, velocity and current angle),
            the currnet path that the car should follow
        and then calculates the steering angle requiered to course correct according to the car's velocity
        in order to update the data in the controller it's update methods need to be called.]
    
    Returns:
        calculate_steering [double] -- [the requierd steering angle in radians to course correct]
    """    

    def __init__(self, car_length=3, kdd=2.8, max_steering_angle=24): # initializes the controller and sets the initial values
        self._car_length = car_length
        self._kdd = kdd
        self._max_steering_angle = max_steering_angle*2*math.pi/360
        self.path = np.poly1d([0,0,0,0])
        self.coordinates = []
        self.orientation = 0 
        self.velocity = 0
            
    def update_state(self, dx, dy, v, d_orientation): # update the current state of the car in the controller for calculations
        self.coordinates = [self.coordinates[0] + dx,self.coordinates[1] + dy]
        self.velocity = v
        self.orientation += (d_orientation*2*math.pi/360)

    def reset_state(self, v): # update the current state of the car in the controller for calculations
        self.coordinates = [0, 0]
        self.velocity = v
        self.orientation = 0

    def update_path(self, path, v): # update the path for the controller for calculations
        self.path = np.poly1d(path)
        self.reset_state(v)

    def calculate_steering(self): # calc the needed steering angle to course correct to the next waypoint
        ld = self.velocity * self._kdd
        look_ahead_point = self._calculate_look_ahead_point(ld)

        alpha = math.atan2(look_ahead_point[1] - self.coordinates[1], look_ahead_point[0] - self.coordinates[0]) - self.orientation # error angle
        delta = math.atan2(2*self._car_length*math.sin(alpha), ld)
        return (max(delta, -self._max_steering_angle) if (delta < 0) else min(delta, self._max_steering_angle))/(2*math.pi/360)

    def _calculate_look_ahead_point(self, ld):
        x_vec = solve((self.path[3]*x**3 + self.path[2]*x**2 + self.path[1]*x + self.path[0] - self.coordinates[1])**2 + (x - self.coordinates[0])**2 - ld**2, x)
        for p in x_vec:
            if not np.iscomplex(x):
                if p > self.coordinates[0]:
                    return [p, self.path(p)]



