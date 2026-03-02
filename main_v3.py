import pygame
import numpy as np
import random
import math

def rotate_point_around_origin(x,y,theta):
    return (
        x * np.cos(theta) + y * -1 * np.sin(theta),
        x * np.sin(theta) + y *      np.cos(theta)
    )

points = [(-.5,-.1), (+.5,-.1), (+.4,+.1), (-.4,+.1), (-.5,-.1)]


class PIDController():
    def __init__(self, kp, ki, kd, kp_y=None, ki_y=None, kd_y=None, kp_theta=None, ki_theta=None, kd_theta=None):
        self.kp = kp
        self.ki = ki 
        self.kd = kd 

        self.kp_y = kp_y if kp_y is not None else kp
        self.ki_y = ki_y if ki_y is not None else ki
        self.kd_y = kd_y if kd_y is not None else kd 

        self.kp_theta = kp_theta if kp_theta is not None else kp 
        self.ki_theta = 0
        self.kd_theta = kd_theta if kd_theta is not None else kd 
        
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
    def calculate_pid(desidered_value, current_value, Kp, Ki, Kd, integral, last_error, dt, integral_limit=10.0):
        error = desidered_value - current_value
        integral += error * dt
        #this is done to prevent the integral from growing too large
        #that would make the drone move too fast and unstable
        integral = max(-integral_limit, min(integral_limit, integral))
        derivative = (error - last_error) / dt
        output = Kp * error + Ki * integral + Kd * derivative
        last_error = error
        return output, integral, last_error 


    def step(self, desidered_y, error_y, theta, dt, desidered_x=0, error_x=0, desidered_theta=0):
        self.thrust_x, self.integral_x, self.last_error_x = self.calculate_pid(desidered_x, error_x, self.kp, self.ki, self.kd, self.integral_x, self.last_error_x, dt)
        target_tilt = np.clip(-0.5 * self.thrust_x, -0.5, 0.5) + desidered_theta
        self.thrust_orientation, self.integral_orientation, self.last_error_orientation = self.calculate_pid(target_tilt, theta, self.kp_theta, self.ki_theta, self.kd_theta, self.integral_orientation, self.last_error_orientation, dt)
        self.thrust_y, self.integral_y, self.last_error_y = self.calculate_pid(desidered_y, error_y, self.kp_y, self.ki_y, self.kd_y, self.integral_y, self.last_error_y, dt)
        
        return self.thrust_x, self.thrust_y, self.thrust_orientation


class Drone:
    def __init__(self, x, y, vx, vy, theta=0, omega=0, mass=1, max_thrust=3, L=0.1 ):
        self.mass = mass
        self.max_thrust = max_thrust * mass  
        self.L = L
        #we set the inertia to be the moment of inertia of a point mass
        #because we are simulating a point mass with 3 motors 
        self.I = mass * (L**2) if L > 0 else 1 
        self.x = x
        self.y = y
        self.vx = vx    
        self.vy = vy
        self.theta = theta
        self.omega = omega
        
        # Speed limits and drag to stabilize drone
        self.max_speed = 5.0
        self.drag = 0.5

        self.controller = PIDController(kp=2, ki=0.3, kd=2.5)
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
        self.circle_speed = 0.5
        self.circle_center = (0, 0)

        # Smoothed thrust values for visual representation
        self.s_main_thrust = 0
        self.s_left_thrust = 0
        self.s_right_thrust = 0

        self.random_points_mode = False
        self.number_of_points = 10
        self.random_points = []

    def is_drone_converged(self): 
        #calculate x and y error 
        error_x = self.desired_x - self.x
        error_y = self.desired_y - self.y
        #check if the drone is close to the desired point
        if abs(error_x) < 0.5 and abs(error_y) < 0.5 :
            return True
        return False
         

    def step(self, main_thrust, left_thrust, right_thrust, dt):  
        if self.circle_mode:
            #we update the path time to create a circular path that changes over time
            self.path_time += dt
            self.desired_x = self.circle_center[0] + self.circle_radius * math.cos(self.circle_speed * self.path_time)
            self.desired_y = self.circle_center[1] + self.circle_radius * math.sin(self.circle_speed * self.path_time)

        # Update internal thrust values from controller
        self.thrust_x, self.thrust_y, self.thrust_orientation = self.controller.step(
            self.desired_y, self.y, self.theta, dt, self.desired_x, self.x, self.desired_orientation
        )

        #we take the maxium of the thrust values to ensure that the motors only push up 
        #otherwise the drone would fall becuase the motors would push down
        #or it will flip
        #but also we take the minimum of 2.0 to ensure that the motors do not push too hard
        #and cause the drone to flip

        self.main_thrust = max(0, min(2.0, 1 * self.thrust_y + main_thrust)) # we add the manual controls to the controller output
        self.left_thrust = max(0, min(2.0, -1 * self.thrust_orientation  + left_thrust))
        self.right_thrust = max(0, min(2.0, 1 * self.thrust_orientation + right_thrust))

        total = self.main_thrust * self.max_thrust #in Newtons
        ax = -np.sin(self.theta) * total / self.mass #in Newtons
        ay = (np.cos(self.theta) * total - 9.81 * self.mass) / self.mass #in Newtons
         
        # Add drag to help the drone brake naturally because the no wind effect was causing the robot to move too freely
        ax -= self.drag * self.vx 
        ay -= self.drag * self.vy

        self.vx += ax * dt # to calculate the velocity it's just taking the derivative of the velocity
        self.vy += ay * dt
        #we want to limit the maxium velocity 
        self.vx = max(-self.max_speed, min(self.max_speed, self.vx))
        self.vy = max(-self.max_speed, min(self.max_speed, self.vy))
        
        self.x += self.vx * dt # to calculate the position it's just taking the derivative of the velocity
        self.y += self.vy * dt



            # Correct torque: Right motor (+L) pushes UP gives positive (CCW) torque
        # Left motor (-L) pushes UP gives negative (CW) torque
        # So torque = (Right - Left) * L
        torque = (self.right_thrust - self.left_thrust) * self.max_thrust * self.L 
         
        self.omega += (torque / self.I) * dt #the inertia is the resistance to change in angular velocity, so by dividing by the inertia we reduce the effect of the torque
        self.theta += self.omega * dt

        # Update smoothed thrust values for visual ONLY visual smoothing (low-pass filter)
        # alpha determines the smoothing speed (lower = smoother/slower)
        alpha = 1
        self.s_main_thrust += (self.main_thrust - self.s_main_thrust) * alpha 
        self.s_left_thrust += (self.left_thrust - self.s_left_thrust) * alpha  
        self.s_right_thrust += (self.right_thrust - self.s_right_thrust) * alpha


    def draw_flame(self, surf, px_per_m, offset_x, thrust_val, size, color):  
    
        w, h = surf.get_width(), surf.get_height()
        def to_px(x, y): 
            return x * px_per_m + w/2, -y * px_per_m + h/2
        
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
            f"Circle Speed: {self.circle_speed:.2f}",
            f"Random Points Mode: {'ON' if self.random_points_mode else 'OFF'}",
            f"L/R Thrust: {self.left_thrust:.2f}/{self.right_thrust:.2f}",
            f"PID X (1-6): {self.controller.kp:.2f}, {self.controller.ki:.2f}, {self.controller.kd:.2f}",
            f"PID Y (NP 1-6): {self.controller.kp_y:.2f}, {self.controller.ki_y:.2f}, {self.controller.kd_y:.2f}",
            f"PID Angle (U-]): {self.controller.kp_theta:.2f}, {self.controller.ki_theta:.2f}, {self.controller.kd_theta:.2f}"
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

        if self.random_points_mode:
            for i, point in enumerate(self.random_points):
                px, py = to_px(point[0], point[1])
                pygame.draw.circle(surf, (255, 0, 0), (px, py), 5)
        
        # Draw current target (manual or circle point)
        tx, ty = to_px(self.desired_x, self.desired_y)
        pygame.draw.circle(surf, (0, 255, 0), (int(tx), int(ty)), 6)
        font = pygame.font.Font(None, 22)
        target_text = font.render(f"Target: ({self.desired_x:.2f}, {self.desired_y:.2f})", True, (0, 255, 0))
        surf.blit(target_text, (tx + 12, ty - 10))

        # Label drone current position
        dx, dy = to_px(self.x, self.y)
        drone_text = font.render(f"Drone: ({self.x:.2f}, {self.y:.2f})", True, (150, 150, 255))
        surf.blit(drone_text, (dx + 12, dy + 15))
  
        self.draw_controller_state(surf, px_per_m) 
        
        # Draw flames using the smoothed thrust values
        self.draw_flame(surf, px_per_m, 0, self.s_main_thrust, 0.6, (255, 120, 0))
        self.draw_flame(surf, px_per_m, -self.L + 0.2, self.s_left_thrust, 0.3, (255, 200, 50))
        self.draw_flame(surf, px_per_m, self.L - 0.2, self.s_right_thrust, 0.3, (255, 200, 50))



# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720)) 
clock = pygame.time.Clock()
running = True
dt = 0
max_thrust = 10 
drone_list = []
px_per_m = 70


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
                    drone_relative_x = d.x - common_center[0]
                    drone_relative_y = d.y - common_center[1]
                    # we need the angle because the drone is rotating and we want to start the circle from the current angle
                    # so atan2 gives us the angle of the vector (dx, dy)
                    # we divide the angle by the circle speed to get the time it takes to complete the circle
                    # because we need to update the desired position in the circle

                    angle = math.atan2(drone_relative_y, drone_relative_x)
                    d.path_time = angle / d.circle_speed

            if event.key == pygame.K_e:
                for drone in drone_list:
                    drone.circle_mode = False
                    drone.random_points_mode = True
                    drone.random_points = []
                    for _ in range(drone.number_of_points):
                        drone.random_points.append((random.uniform(-10, 10), random.uniform(-10, 10)))

            #tune kp, ki, kd with keys
            for d in drone_list:
                if event.key == pygame.K_1:
                    d.controller.kp -= 0.1
                if event.key == pygame.K_2:
                    d.controller.kp += 0.1 
                if event.key == pygame.K_3:
                    d.controller.ki -= 0.1
                if event.key == pygame.K_4:
                    d.controller.ki += 0.1
                if event.key == pygame.K_5:
                    d.controller.kd -= 0.1
                if event.key == pygame.K_6:
                    d.controller.kd += 0.1
                
                # Tune Height (Y) PID with Numpad 1-6
                if event.key == pygame.K_KP1:
                    d.controller.kp_y -= 0.1
                if event.key == pygame.K_KP2:
                    d.controller.kp_y += 0.1
                if event.key == pygame.K_KP3:
                    d.controller.ki_y -= 0.1
                if event.key == pygame.K_KP4:
                    d.controller.ki_y += 0.1
                if event.key == pygame.K_KP5:
                    d.controller.kd_y -= 0.1
                if event.key == pygame.K_KP6:
                    d.controller.kd_y += 0.1
                
                # Tune Orientation (Theta) PID with U/I, O/P, [/]
                if event.key == pygame.K_u:
                    d.controller.kp_theta -= 0.1
                if event.key == pygame.K_i:
                    d.controller.kp_theta += 0.1
                if event.key == pygame.K_o:
                    d.controller.ki_theta -= 0.1
                if event.key == pygame.K_p:
                    d.controller.ki_theta += 0.1
                if event.key == pygame.K_LEFTBRACKET:
                    d.controller.kd_theta -= 0.1
                if event.key == pygame.K_RIGHTBRACKET:
                    d.controller.kd_theta += 0.1
                
                # Tune circle speed with keys 8 and 9
                if event.key == pygame.K_8:
                    d.circle_speed -= 0.1
                if event.key == pygame.K_9:
                    d.circle_speed += 0.1
            


    for drone in drone_list:
        if drone.random_points_mode and len(drone.random_points) > 0:
            target = drone.random_points[0]
            drone.desired_x = target[0]
            drone.desired_y = target[1]
            if drone.is_drone_converged(): 
                drone.random_points.pop(0)
        elif drone.random_points_mode and len(drone.random_points) == 0:
            drone.random_points_mode = False

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






