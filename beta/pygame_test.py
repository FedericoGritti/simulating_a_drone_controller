import pygame
from drone import Drone

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

drone = Drone(0,0,1,2,1)

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False
            if event.key == pygame.K_SPACE:
                # This code executes once every time the space key is pressed 
                pass
            

    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        # This code will execute for every frame in which the given key is pressed
        pass

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("black")

    drone.drawOnSurface(screen, 100)

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000
    
    drone.step(dt)
    print(drone.y)

pygame.quit()

