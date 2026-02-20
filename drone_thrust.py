import pygame

class Drone:
    def __init__(self, x, y, vx, vy, mass, max_thrust=3):
        
        # Object parameters (will not change)
        self.mass = mass # in [kg]
        self.max_thrust = max_thrust * mass # in [N]
        self.thrust = 0


        # State variables (these will change during the life of the object)
        self.x = x # in [m]
        self.y = y
        self.vx = vx # in [m/s]
        self.vy = vy

    def step(self, thrust, dt):
        ''' steps the simulation of this ball. dt is in seconds '''

        # Forces are represented in Newtons. 
        fx = 0
        fy = -9.81 * self.mass
        ax = fx / self.mass 
        ay = fy / self.mass + thrust * self.max_thrust * dt
        self.thrust = thrust
        self.vx = self.vx + ax * dt
        self.vy = self.vy + ay * dt
        self.x = self.x + self.vx * dt
        self.y = self.y + self.vy * dt


    def drawOnSurface(self, surf, px_per_m):
        """ Draws this drone on a pygame Surface """
        w, h = surf.get_width(), surf.get_height()

        def to_pixel_coords(x,y):
            """ Takes coordinates in meters and returns coordinates in pixels """
            xp = x*px_per_m + w/2
            yp = -y*px_per_m + h/2
            return xp,yp

        # pygame.draw.circle(surface = surf, 
        #                    color = (100,100,255), 
        #                    center = to_pixel_coords(self.x,self.y),
        #                    radius = 0.5 * px_per_m)

        rect_w_px = int(0.5 * px_per_m) # we multiply by px_per_m to convert from meters to pixels
        rect_h_px = int(0.2 * px_per_m)
        rect_x_px, rect_y_px = to_pixel_coords(self.x,self.y) # we convert the center of the rectangle from meters to pixels
        rect = pygame.Rect(0,0,rect_w_px,rect_h_px) #0,0 is the top left corner of the rectangle, we will move it later
        rect.center = (rect_x_px, rect_y_px)
        pygame.draw.rect(surf, (100,100,255), rect)

        if self.thrust == 1:
            pygame.draw.line(surf, (255,0,0), start_pos = to_pixel_coords(self.x,self.y), end_pos=to_pixel_coords(self.x, self.y-0.5),width = 10)
        else:
            pygame.draw.line(surf, (0,0,0), start_pos = to_pixel_coords(self.x,self.y), end_pos=to_pixel_coords(self.x, self.y-0.5))