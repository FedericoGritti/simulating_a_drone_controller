import pygame
import numpy as np

def rotate_point_around_origin(x,y,theta):
    return (
        x * np.cos(theta) + y * -1 * np.sin(theta),
        x * np.sin(theta) + y *      np.cos(theta)
    )

points = [(-.5,-.1), (+.5,-.1), (+.4,+.1), (-.4,+.1), (-.5,-.1)]

class Drone:
    def __init__(self, x, y, vx, vy,theta=1,omega=1, mass=1, max_thrust=3, L=1):
        
        # Object parameters (will not change)
        self.mass = mass # in [kg]
        self.max_thrust = max_thrust * mass # in [N]
        self.L = L # in [m] - distance from center to propeller
        self.thrust = 0
        self.omega = omega # in [rad/s]
        self.theta = theta # in [rad]

        # State variables (these will change during the life of the object)
        self.x = x # in [m]
        self.y = y
        self.vx = vx # in [m/s]
        self.vy = vy

    def step(self, thrust_l, thrust_r, dt):
        ''' steps the simulation of this ball. dt is in seconds '''

        # Forces are represented in Newtons. 
        fxt = - np.sin(self.theta) * (thrust_l + thrust_r) * self.max_thrust
        fyt = np.cos(self.theta) * (thrust_l + thrust_r) * self.max_thrust
        fx = fxt
        fy = -9.81 * self.mass + fyt
        ax = fx / self.mass 
        ay = fy / self.mass 
        self.thrust = thrust_l + thrust_r
        self.thrust_l = thrust_l
        self.thrust_r = thrust_r
    

        self.vx = self.vx + ax * dt
        self.vy = self.vy + ay * dt
        self.x = self.x + self.vx * dt
        self.y = self.y + self.vy * dt

        torque = (thrust_r - thrust_l) * self.L * self.max_thrust / self.mass # in [N*m/kg] = [m/s^2]
        self.omega = self.omega + torque * dt
        self.theta = self.theta + self.omega * dt



    def drawOnSurface(self, surf, px_per_m):
        """ Draws this drone on a pygame Surface """
        w, h = surf.get_width(), surf.get_height()

        def to_pixel_coords(x,y):
            """ Takes coordinates in meters and returns coordinates in pixels """
            xp = x*px_per_m + w/2
            yp = -y*px_per_m + h/2
            return xp,yp

        points_list = [rotate_point_around_origin(x,y,self.theta) for x,y in points]
        points_list = [to_pixel_coords(x+self.x,y+self.y) for x,y in points_list]
        pygame.draw.polygon(surf, (100,100,255), points_list)

    
        if self.thrust_r == 1:
            start_1 = self.x + 0.5 * np.sin(self.theta)
            end_1 = self.x - 0.5 * np.sin(self.theta)
            end_2 = self.y + 0.1 * np.cos(self.theta)
            start_2 = self.y - 0.5 * np.cos(self.theta)
            start_3 = self.x + 0.2 * np.sin(self.theta)
            end_3 = self.x - 0.5 * np.sin(self.theta)
            pygame.draw.line(surf, (255,0,0), start_pos = to_pixel_coords(start_1,self.y), end_pos=to_pixel_coords(end_1,self.y),width = 10)  
            pygame.draw.line(surf, (255,0,0), start_pos = to_pixel_coords(self.x,start_2), end_pos=to_pixel_coords(self.x,end_2),width = 10) 
            pygame.draw.line(surf, (255,0,0), start_pos = to_pixel_coords(start_3,self.y), end_pos=to_pixel_coords(end_3,self.y),width = 10) 
        elif self.thrust_l == 2:

            pygame.draw.line(surf, (255,0,0), start_pos = to_pixel_coords(start_1,self.y), end_pos=to_pixel_coords(end_1,self.y),width = 10)  
            pygame.draw.line(surf, (255,0,0), start_pos = to_pixel_coords(self.x,start_2), end_pos=to_pixel_coords(self.x,end_2),width = 10) 
            pygame.draw.line(surf, (255,0,0), start_pos = to_pixel_coords(start_3,self.y), end_pos=to_pixel_coords(end_3,self.y),width = 10) 
        else:
            pygame.draw.line(surf, (0,0,0), start_pos = to_pixel_coords(self.x,self.y), end_pos=to_pixel_coords(self.x, self.y-0.5))r