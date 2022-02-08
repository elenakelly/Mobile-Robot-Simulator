from turtle import Screen, update
from numpy import disp
import random
import pygame 


pygame.init()

#create screen 
display = pygame.display.set_mode((800,600))
pygame.display.update()

pygame.display.set_caption("Mobile Robot Simulator")
icon = pygame.image.load('icon.png')
pygame.display.set_icon(icon)


#robot
robotimg = pygame.image.load('vacuum.png')
robotx = random.uniform(0,400)
roboty = random.uniform(0,400)

robotx_change = 0
roboty_change = 0

def robot (x,y):
    display.blit(robotimg, (x,y))


#game loop
open = True
while open:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            open = False

        if event.type == pygame.KEYDOWN:
            print("press")
            if event.key == pygame.K_a:
                print("Left")
                robotx_change = -1
            if event.key == pygame.K_d:
                print("right")
                robotx_change = 1
            if event.key == pygame.K_w:
                print("right")
                roboty_change = -1
            if event.key == pygame.K_s:
                print("right")
                roboty_change = 1
        
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_a or event.key == pygame.K_d or event.key == pygame.K_w or event.key == pygame.K_s:
                print("release")
                robotx_change = 0
                roboty_change = 0
    
    
    display.fill((255,255,255))

    robotx += robotx_change
    roboty += roboty_change
    robot(robotx,roboty)
    pygame.display.update()        

pygame.quit()
quit()            