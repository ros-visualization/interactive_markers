import pygame, sys
from pygame.locals import *
pygame.init()

screen = pygame.display.set_mode((640, 480))

s = pygame.Surface((100, 150), flags=SRCALPHA)
pygame.draw.rect(s, (0,255,0), (0,0,100,150))
r = pygame.transform.rotate(s, 45)
screen.blit(r, (100, 100))

pygame.display.flip()

while True:

    for event in pygame.event.get():

        # exit on ESC key
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                sys.exit()

