import pygame
import numpy as np
import random
import math

# def rotate_point_around_origin(x, y, theta):
#     c, s = np.cos(theta), np.sin(theta)
#     return x * c - y * s, x * s + y * c

def rotate_point_around_origin(x,y,theta):
    return (
        x * np.cos(theta) + y * -1 * np.sin(theta),
        x * np.sin(theta) + y *      np.cos(theta)
    )

points = [(-.5,-.1), (+.5,-.1), (+.4,+.1), (-.4,+.1), (-.5,-.1)]


class PIDController():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral_x = 0
        self.last_error_x = 0
        self.thrust_x = 0

        self.integral_y = 0
        self.last_error_y = 0
        self.thrust_y = 0

        self.integral_orientation = 0
        self.last_error_orientation = 0
        self.thrust_orientation = 0

        self.main_thrust = 0
        self.left_thrust = 0
        self.right_thrust = 0
        self.is_on = True

    @staticmethod
    def calculate_pid(desidered_value, current_value, Kp, Ki, Kd, integral, last_error, dt):
        error = desidered_value - current_value
        integral += error * dt
        derivative = (error - last_error) / dt
        output = Kp * error + Ki * integral + Kd * derivative
        last_error = error
        return output, integral, last_error 


    def step(self, desidered_y, error_y, theta, dt, desidered_x=0, error_x=0, desidered_theta=0):
        self.thrust_x, self.integral_x, self.last_error_x = self.calculate_pid(desidered_x, error_x, self.kp, self.ki, self.kd, self.integral_x, self.last_error_x, dt)
        target_tilt = np.clip(-0.5 * self.thrust_x, -0.4, 0.4) + desidered_theta
        self.thrust_orientation, self.integral_orientation, self.last_error_orientation = self.calculate_pid(target_tilt, theta, self.kp, self.ki, self.kd, self.integral_orientation, self.last_error_orientation, dt)
        self.thrust_y, self.integral_y, self.last_error_y = self.calculate_pid(desidered_y, error_y, self.kp, self.ki, self.kd, self.integral_y, self.last_error_y, dt)
        
        return self.thrust_x, self.thrust_y, self.thrust_orientation


class Drone:
    def __init__(self, x, y, vx, vy, theta=0, omega=0, mass=1, max_thrust=3, L=0.1 ):
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
        #we change the pid values to value optimized for this drone
        self.controller = PIDController(kp=2, ki=0.1, kd=1)
        self.desired_x = 0
        self.desired_y = 0
        self.desired_orientation = 0
        self.thrust_x = 0
        self.thrust_y = 0
        self.thrust_orientation = 0
        self.main_thrust = 0
        self.left_thrust = 0
        self.right_thrust = 0   
        
        # Circular path parameters
        self.path_time = 0
        self.circle_mode = False
        self.circle_radius = 3.0
        self.circle_speed = 1
        self.circle_center = (0, 0)

        # Smoothed thrust values for visual representation
        self.s_main_thrust = 0
        self.s_left_thrust = 0
        self.s_right_thrust = 0

    def reset_integrals(self):
        """ Reset all integrals to avoid windup when target changes """
        self.controller.integral_x = 0
        self.controller.integral_y = 0
        self.controller.integral_orientation = 0

    def is_centered(self):
        """ Returns True if the drone is inside the square, nearly horizontal and nearly still """
        pos_ok = abs(self.x - self.desired_x) < 0.15 and abs(self.y - self.desired_y) < 0.45
        angle_ok = abs(self.theta) < 0.05   # nearly horizontal
        speed_ok = abs(self.vx) < 0.1 and abs(self.vy) < 0.1  # nearly still
        return pos_ok and angle_ok and speed_ok

    def step(self, main_thrust, left_thrust, right_thrust, dt):  
        if self.circle_mode:
            self.path_time += dt
            self.desired_x = self.circle_center[0] + self.circle_radius * math.cos(self.circle_speed * self.path_time)
            self.desired_y = self.circle_center[1] + self.circle_radius * math.sin(self.circle_speed * self.path_time)

        # Update internal thrust values from controller
        self.thrust_x, self.thrust_y, self.thrust_orientation = self.controller.step(
            self.desired_y, self.y, self.theta, dt, self.desired_x, self.x, self.desired_orientation
        )

        #we take the maxium of the thrust values to ensure that the motors only push up
        #otherwise the drone would fall becuase the motors would push down
        # gravity feedforward: constant base thrust to counteract gravity so PID only handles corrections
        gravity_ff = (self.mass * 9.81) / self.max_thrust
        self.main_thrust = max(0, gravity_ff + 0.5 * self.thrust_y + main_thrust)
        self.left_thrust = max(0, -0.5 * self.thrust_orientation + left_thrust)
        self.right_thrust = max(0, 0.5 * self.thrust_orientation + right_thrust)

        total = self.main_thrust * self.max_thrust 
        ax = -np.sin(self.theta) * total / self.mass
        ay = (np.cos(self.theta) * total - 9.81 * self.mass) / self.mass
         
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Correct torque: Right motor (+L) pushes UP gives positive (CCW) torque
        # Left motor (-L) pushes UP gives negative (CW) torque
        # So torque = (Right - Left) * L
        torque = (self.right_thrust - self.left_thrust) * self.max_thrust * self.L
         
        self.omega += (torque / self.I) * dt
        self.theta += self.omega * dt

        # Update smoothed thrust values for visual smoothing (low-pass filter)
        # alpha determines the smoothing speed (lower = smoother/slower)
        alpha = 0.15
        self.s_main_thrust += (self.main_thrust - self.s_main_thrust) * alpha
        self.s_left_thrust += (self.left_thrust - self.s_left_thrust) * alpha
        self.s_right_thrust += (self.right_thrust - self.s_right_thrust) * alpha

    def draw_flame(self, surf, px_per_m, offset_x, thrust_val, size, color): 
        #avoid small flames
        # if thrust_val < 0.1:
        #     return
        w, h = surf.get_width(), surf.get_height()
        def to_px(x, y): 
            return x * px_per_m + w/2, -y * px_per_m + h/2
        
        # More natural scaling
        # s = size * (0.3 + 0.7 * min(thrust_val, 2.0))
        s = size * thrust_val  
        
        pts = [(offset_x - s*0.4, -0.1), (offset_x + s*0.4, -0.1), (offset_x, -0.1 - s)]
        
        rotated_pts = [] 
        for px, py in pts:
            rx, ry = rotate_point_around_origin(px, py, self.theta)
            rotated_pts.append(to_px(rx + self.x, ry + self.y))
        
        pygame.draw.polygon(surf, color, rotated_pts)

    def draw_controller_state(self, surf, px_per_m):
        font = pygame.font.Font(None, 24) # Smaller font to fit more info
        states = [
            f"Pos: ({self.x:.2f}, {self.y:.2f})",
            f"Desired: ({self.desired_x:.2f}, {self.desired_y:.2f})",
            f"Thrust Y: {self.thrust_y:.2f}",
            f"Orientation: {self.theta:.2f}",
            f"Circle Mode: {'ON' if self.circle_mode else 'OFF'}",
            f"L/R Thrust: {self.left_thrust:.2f}/{self.right_thrust:.2f}"
        ]
        for i, text_str in enumerate(states):
            text = font.render(text_str, True, (255, 255, 255))
            surf.blit(text, (10, 10 + i * 20))

    def drawOnSurface(self, surf, px_per_m):

        # scale_x = self.L / 0.5 
        scale_x = 1
        w, h = surf.get_width(), surf.get_height()
        def to_px(x,y):
            return x*px_per_m + w/2, -y*px_per_m + h/2
            
        scaled_points = [(x * scale_x, y) for x, y in points]
        body_points = [rotate_point_around_origin(x,y,self.theta) for x,y in scaled_points]
        body_pixels = [to_px(px + self.x, py + self.y) for px, py in body_points]
        
        pygame.draw.polygon(surf, (100,100,255), body_pixels)
        
        # Draw path if in circle mode
        if self.circle_mode:
            cx_px, cy_px = to_px(self.circle_center[0], self.circle_center[1])
            r_px = int(self.circle_radius * px_per_m)
            pygame.draw.circle(surf, (50, 50, 50), (int(cx_px), int(cy_px)), r_px, 1)

        self.draw_controller_state(surf, px_per_m) 
        
        # Draw flames using the smoothed thrust values
        self.draw_flame(surf, px_per_m, 0, self.s_main_thrust, 0.6, (255, 120, 0))
        self.draw_flame(surf, px_per_m, -self.L + 0.2, self.s_left_thrust, 0.3, (255, 200, 50))
        self.draw_flame(surf, px_per_m, self.L - 0.2, self.s_right_thrust, 0.3, (255, 200, 50))


def new_target(px_per_m, screen_w, screen_h):
    """ Generate a random target position within the visible screen area """
    margin = 2  # meters from edge
    x_range = (screen_w / 2) / px_per_m - margin
    y_range = (screen_h / 2) / px_per_m - margin
    tx = random.uniform(-x_range, x_range)
    ty = random.uniform(-y_range, y_range)
    return tx, ty

def draw_target_square(surf, px_per_m, tx, ty, size=0.8):
    """ Draw the target square centered at (tx, ty) in world coordinates """
    w, h = surf.get_width(), surf.get_height()
    # convert world coords to pixel coords
    px = tx * px_per_m + w / 2
    py = -ty * px_per_m + h / 2
    size_px = size * px_per_m
    rect = pygame.Rect(px - size_px / 2, py - size_px / 2, size_px, size_px)
    pygame.draw.rect(surf, (0, 255, 0), rect, 2)  # green outline


# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720)) 
clock = pygame.time.Clock()
running = True
dt = 0
max_thrust = 10 
drone_list = []
px_per_m = 70

# target square state
target_x, target_y = None, None  # no target initially
square_mode = False  # whether K mode is active


while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False 
            if event.key == pygame.K_SPACE: 
                drone_list.append(Drone(x=0,y=0,vx=random.uniform(-1 ,1),vy=random.uniform(-1,1),theta=0,omega= random.uniform(-1,1),max_thrust=max_thrust, L=0.5)  )

            if event.key == pygame.K_c:
                drone_list = []
                square_mode = False
                target_x, target_y = None, None
  
            # K key: clear all drones, spawn one, start square chasing mode
            if event.key == pygame.K_k:
                drone_list = []
                drone_list.append(Drone(x=random.uniform(-3, 3), y=random.uniform(-3, 3),
                                        vx=0, vy=0, theta=0, omega=0,
                                        max_thrust=max_thrust, L=0.5))
                target_x, target_y = new_target(px_per_m, screen.get_width(), screen.get_height())
                drone_list[0].desired_x = target_x
                drone_list[0].desired_y = target_y
                square_mode = True

            #this is for pid control
            for d in drone_list:
                if event.key == pygame.K_UP: 
                    d.desired_y += 1
                if event.key == pygame.K_DOWN:
                    d.desired_y -= 1 
                if event.key == pygame.K_LEFT:
                    d.desired_x -= 1 
                if event.key == pygame.K_RIGHT: 
                    d.desired_x += 1
            if event.key == pygame.K_t:
                common_center = (0, 2)
                for d in drone_list:
                    d.circle_mode = True
                    d.circle_center = common_center
                    # Calculate path_time based on current position relative to center
                    # to avoid jumps when activating circle mode
                    dx = d.x - common_center[0]
                    dy = d.y - common_center[1]
                    d.path_time = math.atan2(dy, dx) / d.circle_speed


    keys = pygame.key.get_pressed() 
    if keys[pygame.K_z]:
        px_per_m -= 1
    if keys[pygame.K_x]:
        px_per_m += 1

    # check if drone has centered on the square
    if square_mode and drone_list:
        drone = drone_list[0]
        if drone.is_centered():
            # generate new target and send drone there
            target_x, target_y = new_target(px_per_m, screen.get_width(), screen.get_height())
            drone.desired_x = target_x
            drone.desired_y = target_y
            drone.controller.integral_y = 0  # reset only y integral to avoid upward jump

    screen.fill("black")

    # draw target square if in square mode
    if square_mode and target_x is not None:
        draw_target_square(screen, px_per_m, target_x, target_y)

    for d in drone_list: 
        d.drawOnSurface(screen, px_per_m)
    pygame.display.flip()
    dt = clock.tick(60) / 1000
    
    if drone_list:
        for d in drone_list:
            d.step( d.controller.main_thrust, d.controller.left_thrust, d.controller.right_thrust, dt)

pygame.quit()