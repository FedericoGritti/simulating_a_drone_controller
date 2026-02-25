import pygame
from drone_2 import Drone

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

# create a drone at position (0,0), not rotating, not moving, 
# with mass 1kg, max thrust 10N per thruster, and arm length 0.5m
drone = Drone(0, 0, 0, 0, 0, 0, 1, 10, 0.5)

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False

    # check which keys are currently held down
    # W fires the left thruster, E fires the right thruster
    keys = pygame.key.get_pressed()
    thrust_l = 1 if keys[pygame.K_w] else 0
    thrust_r = 1 if keys[pygame.K_e] else 0

    # fill the screen with black to wipe away anything from last frame
    screen.fill("black")

    # draw the drone on the screen
    # second parameter is pixels per meter (zoom level)
    drone.drawOnSurface(screen, 50)

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics
    dt = clock.tick(60) / 1000

    # step the simulation forward with the current thrust values
    drone.step(thrust_l, thrust_r, dt)

pygame.quit()