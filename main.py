
import pygame
import time
import math
from utils import blit_rotate_center,blit_text_center
import random

pygame.font.init()

#images
BACKGROUND = pygame.image.load("images/background.png")
ROBOT = pygame.image.load("images/vacuum.png")
ICON = pygame.image.load('images/icon.png')

#main sceen 
WIDTH, HEIGHT = 800, 600
SCREEN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Mobile Robot Simulator")
pygame.display.set_icon(ICON)
pygame.rect.Rect
FPS = 60
MAIN_FONT = pygame.font.SysFont("comicsans", 44)


#Robot movement
class RobotMove:
    def __init__(self, max_vel, rotation_vel):
        self.img = self.IMG
        self.max_vel = max_vel
        self.vel = 0
        self.rotation_vel = rotation_vel
        self.angle = 0
        self.x, self.y = self.START_POS
        self.acceleration = 0.1

    def rotate(self, left=False, right=False):
        if left:
            self.angle += self.rotation_vel
        elif right:
            self.angle -= self.rotation_vel

    def draw(self, win):
        blit_rotate_center(win, self.img, (self.x, self.y), self.angle)

    def move_forward(self):
        self.vel = min(self.vel + self.acceleration, self.max_vel)
        self.move()
    
    def move_backward(self):
        self.vel = max(self.vel - self.acceleration, -self.max_vel/2)
        self.move()
    
    def stop(self):
        self.vel = 0
        self.angle = 0
        self.move()

    def move(self):
        radians = math.radians(self.angle)
        vertical = math.cos(radians) * self.vel
        horizontal = math.sin(radians) * self.vel

        self.y -= vertical
        self.x -= horizontal
        
    
    

    

 

class PlayRobot(RobotMove):
    IMG = ROBOT
    START_POS = (random.uniform(0,735),random.uniform(0,535))
    
    def collide(self):
        #hit the wall
        if self.x <= 0:
            self.x = 0
        elif self.x >= 736:
            self.x = 736

        if self.y <= 0:
            self.y = 0
        elif self.y >= 536:
            self.y = 536


def draw(screen,images, player_robot):
    
    for img,pos in images:
        screen.blit(img,pos)



    vel_text = MAIN_FONT.render(
        f"Vel: {round(player_robot.vel, 1)}", 1, (255, 255, 255))
    screen.blit(vel_text, (10, HEIGHT - vel_text.get_height() - 10))

    player_robot.draw(screen)
    pygame.display.update()
  



run = True
clock = pygame.time.Clock()
images = [(BACKGROUND, (0, 0)), (WALL, (342,142)),(WALL, (342,342))]
player_robot = PlayRobot(4, 4)


#visualization
while run:
    clock.tick(FPS)

    draw(SCREEN,images, player_robot)


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
            

    keys = pygame.key.get_pressed()
    moved = False

    #left wheel
    if keys[pygame.K_w]:
        player_robot.rotate(left=True)
    if keys[pygame.K_s]:
        player_robot.rotate(right=True)
    
    #right wheel
    if keys[pygame.K_o]:
        player_robot.rotate(right=True)
    if keys[pygame.K_l]:
        player_robot.rotate(left=True)
    
    #both wheels
    if keys[pygame.K_t]:
        moved = True
        player_robot.move_forward()
    if keys[pygame.K_g]:
        moved = True
        player_robot.move_backward()
    
    #stop
    if keys[pygame.K_x]:
        player_robot.stop()

    player_robot.collide()
 



pygame.quit()
#test
