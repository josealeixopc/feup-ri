# Python imports
import random

# Library imports
import pygame
from pygame.key import *
from pygame.locals import *
from pygame.color import *

# pymunk imports
import pymunk
import pymunk.pygame_util

def setup_space():
    space = pymunk.Space()
    space.gravity = 0.0, 0.0
    # space.damping = 0.99 # No damping for now
    return space

if __name__ == '__main__':
    
    pygame.init()
    screen = pygame.display.set_mode((600, 600))
    clock = pygame.time.Clock()
    options = pymunk.pygame_util.DrawOptions(screen)


    running = True

    s = setup_space()

    radius = 25
    mass = 10

    b = pymunk.Body(mass, pymunk.moment_for_circle(mass, 0, radius, (0,0)))
    c1 = pymunk.Circle(b, radius)

    c1.body.position = 100, 100

    s.add(b, c1)
    
    wheelMass = 1
    wheelWidth = 10
    wheelHeight = 20

    wheelBodyRight = pymunk.Body(wheelMass, pymunk.moment_for_box(wheelMass,(wheelWidth, wheelHeight)))
    wheelRight = pymunk.Poly(wheelBodyRight, [(wheelWidth/2, wheelHeight/2),(wheelWidth/2, -wheelHeight/2), (-wheelWidth/2, -wheelHeight/2), (-wheelWidth/2, wheelHeight/2)])

    wheelRight.color = (255, 0, 0)

    wheelRight.body.position = 100+radius,100

    s.add(wheelRight, wheelBodyRight)


    j = pymunk.PinJoint(c1.body, wheelRight.body)

    s.add(j)
 
    # options = pymunk.SpaceDebugDrawOptions()
    while running:
        screen.fill((255,255,255))


        # c1.body.apply_force_at_local_point((0, 100), (radius, 0))
        wheelRight.body.apply_force_at_local_point((0, -1), (0, 0))    


        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

        s.step(1/1000)
        s.debug_draw(options)
        pygame.display.flip()

        clock.tick(1000)

        print(c1.body.angular_velocity)
