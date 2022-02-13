import pygame
from utils import blit_rotate_center
import random
import numpy as np

#initialisation of game
pygame.font.init()

#images
BACKGROUND = pygame.image.load("images/background.png")
ROBOT = pygame.image.load("images/vacuum.png")
WALL = pygame.image.load("images/wall.png")
ICON = pygame.image.load('images/icon.png')

#main sceen 
WIDTH, HEIGHT = 800, 600 #dimentions
SCREEN = pygame.display.set_mode((WIDTH, HEIGHT))
#window setting
pygame.display.set_caption("Mobile Robot Simulator") 
pygame.display.set_icon(ICON)
pygame.rect.Rect
MAIN_FONT = pygame.font.SysFont("comicsans", 22)


#Robot movement
class RobotMove:
    def __init__(self):
        self.trail_set = []

        self.img = self.IMG #image
        self.x = self.START_POS[0] #starting x
        self.y = self.START_POS[1] #starting y

        self.m2p = 3779.52 #meters to pixels
        self.vl = 0  #left velocity
        self.vr = 0 #right velocity
        self.theta = 0
        self.speed = 0.001
        distance = 64
        self.l = int(distance/2) #distance between the centers of the two wheels
        
        self.changeX = self.x + self.l
        self.changeY = self.y


    def move(self,keys,dt):
        
        #setting the buttons
        if keys[0] == 1 :
            self.vl += self.speed 
        if keys[1] == 1 :
            self.vl -= self.speed
        if keys[2] == 1 :
            self.vr += self.speed
        if keys[3] == 1:
            self.vr -= self.speed
        if  keys[4] == 1:
            self.vl == self.vr
            self.vr += self.speed
            self.vl += self.speed
        if  keys[5] == 1:
            self.vl == self.vr
            self.vr -= self.speed
            self.vl -= self.speed
        if keys[6] == 1:
            self.vl = 0
            self.vr = 0
            

          # check model
        if self.vr != 0 or self.vl != 0:
            if self.vl == self.vr:
                self.x += ((self.vl+self.vr)/2) * np.cos(self.theta) * dt
                self.y -= ((self.vl+self.vr)/2) * np.sin(self.theta) * dt
                R = np.inf
                w = 0   
            else:
                R = self.l * (self.vl + self.vr) / (self.vr - self.vl)
                w = (self.vr - self.vl) / (self.l*2)

            # Computation of ICC
                ICC = [self.x - R * np.sin(self.theta), self.y + R * np.cos(self.theta)]
                rotation = np.transpose(np.matmul(
                np.array([[np.cos(w * dt), -np.sin(w * dt), 0],
                          [np.sin(w * dt), np.cos(w * dt), 0],
                          [0, 0, 1]]),
                np.transpose(np.array([self.x - ICC[0], self.y - ICC[1], self.theta]))) + np.array(
                [ICC[0], ICC[1], w * dt])).transpose()   
                
                self.x = rotation[0]
                self.y = rotation[1]
                self.theta = rotation[2]
                self.rotate(self.theta)
    

    def rotate(self,angle):
        self.changeX = self.x + np.cos(angle) * self.l # Rotatation from the x-axis
        self.changeY = self.y + np.sin(angle) * self.l # Rotatation from the y-axis

    #draw and rotate the image
    def draw(self, win):
        blit_rotate_center(win, self.img, (self.x, self.y), self.theta)
    
 

class PlayRobot(RobotMove):
    IMG = ROBOT
    START_POS = (random.uniform(0,735),random.uniform(0,535)) #start at random potition
    trail_set =[]
    
    def collide(self):
        self.boundaryX = WIDTH - 64
        self.boundaryY = HEIGHT- 64
        #hit the wall
        if self.x <= 0:
            self.x = 0
        elif self.x >= self.boundaryX:
            self.x = self.boundaryX

        if self.y <= 0:
            self.y = 0
        elif self.y >= self.boundaryY :
            self.y = self.boundaryY




#setting the enviroment
class Envir:
        def __init__(self,dimention):
            #colors
            self.black = (0,0,0)
            self.white = (255,255,255)
            #map_dims
            self.height= dimention[0]
            self.width = dimention[1]
            #window setting
            self.map = pygame.display.set_mode((self.width, self.height))
            #trail
            self.trail_set=[]      
        #line route
        def trail(self,pos):
            for i in range(0,len(self.trail_set)-1):
                pygame.draw.line(self.map,self.white,(self.trail_set[i][0],self.trail_set[i][1]),
                                (self.trail_set[i+1][0],self.trail_set[i+1][1]))
            if self.trail_set.__sizeof__()>10000:
                self.trail_set.pop(0)
            self.trail_set.append(pos)
        #y and x axis
        def robot_frame(self,pos,rotation):
            n = 80
            centerx,centery = pos
            x_axis= (centerx +n*np.cos(-rotation),centery +n*np.sin(-rotation))
            y_axis= (centerx +n*np.cos(-rotation+np.pi/2),centery +n*np.sin(-rotation+np.pi/2))
            pygame.draw.line(self.map,self.black,(centerx,centery),x_axis,3)
            pygame.draw.line(self.map,self.black,(centerx,centery),y_axis,3)
            
        def draw(self,screen,images, player_robot):

            #display images on screen
            for img,pos in images:
                screen.blit(img,pos)
            
            #display left, right velocity and theta on screen
            vel_text = MAIN_FONT.render(
                f"Vl = {player_robot.vl} Vr = {player_robot.vr} theta = {int(np.degrees(player_robot.theta))}", 1, self.white)
            screen.blit(vel_text, (10, HEIGHT - vel_text.get_height() - 40))

            #display robot on screen
            player_robot.draw(screen)
            #pygame.display.update()
        
 


#running game or not
run = True

images = [(BACKGROUND, (0, 0)), (WALL, (342,142)),(WALL, (342,342))]
#the robot 
player_robot = PlayRobot()

#enviroment prints
enviroment = Envir([600,800])



#dt
dt = 1000 
clock = pygame.time.Clock()
FPS = 60

#simulation loop
while run:

    #activate quit button
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    #timer
    clock.tick(FPS)
    
    #activate buttons
    keys = pygame.key.get_pressed()
    key = [keys[pygame.K_w], keys[pygame.K_s], keys[pygame.K_o], keys[pygame.K_l], 
            keys[pygame.K_t], keys[pygame.K_g], keys[pygame.K_x]]

    #run the robot
    activate = player_robot.move(key,dt)
    player_robot.collide()

   
#visualize objects
    enviroment.draw(SCREEN,images,player_robot)
    enviroment.robot_frame((player_robot.x,player_robot.y),player_robot.theta)
    enviroment.trail((player_robot.x,player_robot.y))
    player_robot.draw(enviroment.map)
    pygame.display.update()
    
#exit the game
pygame.quit()

