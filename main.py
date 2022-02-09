from turtle import Screen, distance, update
from numpy import disp
import random
import pygame 
import math


pygame.init()

#create screen 
display = pygame.display.set_mode((800,600))
background = pygame.image.load('background.png')

pygame.display.set_caption("Mobile Robot Simulator")
icon = pygame.image.load('icon.png')
pygame.display.set_icon(icon)


#robot
robotimg = pygame.image.load('vacuum.png')
robotx = random.uniform(0,735)
roboty = random.uniform(0,535)

robotx_change = 0
roboty_change = 0

#wall

wallimg = []
wallx = []
wally = []
wallx_change = []
wally_change = []
num_walls = 10

for i in range(num_walls):

    wallimg.append(pygame.image.load('wall.png'))
    wallx.append(random.uniform(0,800))
    wally.append(random.uniform(0,600))

    
    



def robot (x,y):
    display.blit(robotimg, (x,y))

def wall (x,y,i):
    display.blit(wallimg[i], (x,y))

def isCollision(robotx,roboty,wallx,wally):
    distance =  math.sqrt(math.pow(robotx-roboty,2)+ (math.pow(wallx-wally,2)))
    if distance <27:
        return True 
    


#game loop
open = True
while open:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            open = False


        display.blit(background,(0,0))

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
        
        for i in range(num_walls):
        #Collision
            collision = isCollision(robotx,roboty,wallx[i],wally[i])
            if collision:
                robotx -= 1
                roboty -= 1
            wall(wallx[i],wally[i],i)
    

    robotx += robotx_change
    roboty += roboty_change

    if robotx <= 0:
        robotx = 0
    elif robotx >= 736:
        robotx = 736

    if roboty <= 0:
        roboty = 0
    elif roboty >= 536:
        roboty = 536

    robot(robotx,roboty)
    


    pygame.display.update()        

pygame.quit()
quit()                       
