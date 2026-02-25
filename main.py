import pygame
import numpy as np
import random
def rotate_point_around_origin(x, y, theta):
    c, s = np.cos(theta), np.sin(theta)
    return x * c - y * s, x * s + y * c

points = [(-.5,-.1), (+.5,-.1), (+.4,+.1), (-.4,+.1), (-.5,-.1)]

class PIDController():
    def __init__(self, Kp_height=1, Ki_height=0.1, Kd_height=0.1, Kp_orientation=1, Ki_orientation=0.1, Kd_orientation=0.1):
        self.Kp_height = Kp_height
        self.Ki_height = Ki_height
        self.Kd_height = Kd_height
        self.integral_height = 0
        self.last_error_height = 0
        self.thrust_height = 0
        
        self.Kp_orientation = Kp_orientation
        self.Ki_orientation = Ki_orientation 
        self.Kd_orientation = Kd_orientation
        self.integral_orientation = 0
        self.last_error_orientation = 0
        self.thrust_orientation = 0

        self.main_thrust = 0
        self.left_thrust = 0
        self.right_thrust = 0
        self.is_on = False



    def step(self, desired_height, desired_orientation, error_height, error_orientation, dt):
        error_height = desired_height - error_height
        error_orientation = desired_orientation - error_orientation
        self.integral_height += error_height * dt
        derivative_height = (error_height - self.last_error_height) / dt
        output_height = self.Kp_height * error_height + self.Ki_height * self.integral_height + self.Kd_height * derivative_height
        self.last_error_height = error_height
        self.thrust_height = output_height
        self.integral_orientation += error_orientation * dt
        derivative_orientation = (error_orientation - self.last_error_orientation) / dt
        output_orientation = self.Kp_orientation * error_orientation + self.Ki_orientation * self.integral_orientation + self.Kd_orientation * derivative_orientation
        self.last_error_orientation = error_orientation
        self.thrust_orientation = output_orientation

        return output_height, output_orientation




class Drone:
    def __init__(self, x, y, vx, vy, theta=0, omega=0, mass=1, max_thrust=3, L=0.8, controller=PIDController()):
        self.mass = mass
        self.max_thrust = max_thrust * mass 
        self.L = L
        self.I = mass * (L**2) if L > 0 else 1 
        self.x = x
        self.y = y
        self.vx = vx    
        self.vy = vy
        self.theta = theta
        self.omega = omega
        self.controller = PIDController(Kp_height=1, Ki_height=0, Kd_height=0, Kp_orientation=1, Ki_orientation=0, Kd_orientation=0)
        self.controller.is_on = True
        self.desired_height = 0
        self.desired_orientation = 0
       

    def step(self, main_thrust, left_thrust, right_thrust,dt):
        thrust_height, thrust_orientation = self.controller.step(self.desired_height, self.desired_orientation, self.y, self.theta, dt)
        total = (thrust_height + main_thrust) * self.max_thrust
        
        ax = -np.sin(self.theta) * total / self.mass
        ay = (np.cos(self.theta) * total - 9.81 * self.mass) / self.mass
         
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        torque = ( thrust_orientation + left_thrust - right_thrust) * self.max_thrust * self.L
        self.omega += (torque / self.I) * dt
        self.theta += self.omega * dt

    def draw_flame(self, surf, px_per_m, offset_x, thrust_val, size, color):
        if thrust_val <= 0: return
        w, h = surf.get_width(), surf.get_height()
        def to_px(x, y): 
            return x * px_per_m + w/2, -y * px_per_m + h/2
        
        s = size * (0.5 + 0.5 * thrust_val)
        pts = [(offset_x - s*0.4, -0.1), (offset_x + s*0.4, -0.1), (offset_x, -0.1 - s)]
        
        rotated_pts = []
        for px, py in pts:
            rx, ry = rotate_point_around_origin(px, py, self.theta)
            rotated_pts.append(to_px(rx + self.x, ry + self.y))
        
        pygame.draw.polygon(surf, color, rotated_pts)

    def draw_controller_state(self, surf, px_per_m):
        font = pygame.font.Font(None, 36)
        text = font.render(f"Controller On: {self.controller.is_on}", True, (255, 255, 255))
        # #add a button
        # button = Button(10, 130, "Toggle Controller")
        # button.draw(surf)
        # if button.is_pressed():
        #     self.controller.is_on = not self.controller.is_on
        #draw desired height and orientation
        text = font.render(f"Desired Height: {self.desired_height}", True, (255, 255, 255))
        surf.blit(text, (10, 130))
        text = font.render(f"Desired Orientation: {self.desired_orientation}", True, (255, 255, 255))
        surf.blit(text, (10, 160))
        #current values coefficients of the pid controller  
        text = font.render(f"Kp_height: {self.controller.Kp_height}", True, (255, 255, 255))
        surf.blit(text, (10, 220))
        text = font.render(f"Ki_height: {self.controller.Ki_height}", True, (255, 255, 255))
        surf.blit(text, (10, 250))
        text = font.render(f"Kd_height: {self.controller.Kd_height}", True, (255, 255, 255))
        surf.blit(text, (10, 280))
        text = font.render(f"Kp_orientation: {self.controller.Kp_orientation}", True, (255, 255, 255))
        surf.blit(text, (10, 310))
        text = font.render(f"Ki_orientation: {self.controller.Ki_orientation}", True, (255, 255, 255))
        surf.blit(text, (10, 340))
        text = font.render(f"Kd_orientation: {self.controller.Kd_orientation}", True, (255, 255, 255))
        surf.blit(text, (10, 370))

    def drawOnSurface(self, surf, px_per_m):
        scale_x = self.L / 0.5 
        w, h = surf.get_width(), surf.get_height()
        def to_px(x,y):
            return x*px_per_m + w/2, -y*px_per_m + h/2
        scaled_points = [(x * scale_x, y) for x, y in points]
        body_points = [rotate_point_around_origin(x,y,self.theta) for x,y in scaled_points]
        body_pixels = [to_px(px + self.x, py + self.y) for px, py in body_points]
        pygame.draw.polygon(surf, (100,100,255), body_pixels)
        self.draw_controller_state(surf, px_per_m) 
        self.draw_flame(surf, px_per_m, 0, self.controller.thrust_height , 0.5, (255, 120, 0))      # Main (Big)
        self.draw_flame(surf, px_per_m, -self.L + 0.3, self.controller.thrust_orientation, 0.25, (255, 200, 50)) # Left
        self.draw_flame(surf, px_per_m, self.L - 0.3, self.controller.thrust_orientation, 0.25, (255, 200, 50))  # Right


        

        
# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720)) 
clock = pygame.time.Clock()
running = True
dt = 0
max_thrust = 50
drone_list = []
px_per_m = 100


while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False
            if event.key == pygame.K_SPACE: 
                drone_list.append(Drone(x=0,y=0,vx=random.uniform(-0.5 ,0.5),vy=random.uniform(-0.5,0.5),theta=random.uniform(-1,1),omega= random.uniform(-1,1),max_thrust=max_thrust, L=2 ))

            if event.key == pygame.K_e:
                drone_list = []
 
            #this is for pid control
            #for d in drone_list:
                # if event.key == pygame.K_UP: 
                #     drone_list[0].desired_height += 0.1
                # if event.key == pygame.K_DOWN:
                #     drone_list[0].desired_height -= 0.1
                # if event.key == pygame.K_LEFT:
                #     drone_list[0].desired_orientation += 0.1
                # if event.key == pygame.K_RIGHT: 
                #     drone_list[0].desired_orientation -= 0.1

            #this is for manual control
            for d in drone_list:
                if event.key == pygame.K_UP:
                    d.controller.main_thrust = 0.5
                if event.key == pygame.K_DOWN:
                    d.controller.main_thrust = -0.5
                if event.key == pygame.K_LEFT: 
                    d.controller.left_thrust = 0.3
                if event.key == pygame.K_RIGHT:
                    d.controller.right_thrust = 0.3


    keys = pygame.key.get_pressed() 
    if keys[pygame.K_z]:
        px_per_m -= 1
    if keys[pygame.K_x]:
        px_per_m += 1
    
    screen.fill("black")
    for d in drone_list: 
        d.drawOnSurface(screen, px_per_m)
    pygame.display.flip()
    dt = clock.tick(60) / 1000
    if drone_list:
        for d in drone_list:
            d.step( d.controller.main_thrust, d.controller.left_thrust, d.controller.right_thrust, dt)

pygame.quit()







