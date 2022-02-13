import pygame
from utils import blit_rotate_center
import random
import numpy as np
import math

# initialisation of game
pygame.font.init()

# images
BACKGROUND = pygame.image.load("images/background2.png")
ROBOT = pygame.image.load("images/vacuum.png")
WALL = pygame.image.load("images/wall.png")
ICON = pygame.image.load('images/icon.png')

# main sceen
WIDTH, HEIGHT = 800, 600  # dimentions
SCREEN = pygame.display.set_mode((WIDTH, HEIGHT))
# window setting
pygame.display.set_caption("Mobile Robot Simulator")
pygame.display.set_icon(ICON)
pygame.rect.Rect
MAIN_FONT = pygame.font.SysFont("comicsans", 22)
SENSORS_FONT = pygame.font.SysFont("comicsans", 12)


# SENSORS - Divide circumference by number of sensors
STEP_ANGLE = (math.pi*2) / 12
# -------

# Robot movement


class RobotMove:
    def __init__(self):
        self.trail_set = []

        self.img = self.IMG  # image
        self.x = self.START_POS[0]  # starting x
        self.y = self.START_POS[1]  # starting y

        self.m2p = 3779.52  # meters to pixels
        self.vl = 0  # left velocity
        self.vr = 0  # right velocity
        self.theta = 0
        self.speed = 0.001
        distance = 64
        # distance between the centers of the two wheels
        self.l = int(distance/2)

        self.changeX = self.x + self.l
        self.changeY = self.y

    def move(self, keys, dt):

        # setting the buttons
        if keys[0] == 1:
            self.vl += self.speed
        if keys[1] == 1:
            self.vl -= self.speed
        if keys[2] == 1:
            self.vr += self.speed
        if keys[3] == 1:
            self.vr -= self.speed
        if keys[4] == 1:
            self.vl == self.vr
            self.vr += self.speed
            self.vl += self.speed
        if keys[5] == 1:
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
                ICC = [self.x - R * np.sin(self.theta),
                       self.y + R * np.cos(self.theta)]
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

    def rotate(self, angle):
        # Rotatation from the x-axis
        self.changeX = self.x + np.cos(angle) * self.l
        # Rotatation from the y-axis
        self.changeY = self.y + np.sin(angle) * self.l

    # draw and rotate the image
    def draw(self, win):
        blit_rotate_center(win, self.img, (self.x, self.y), self.theta)


class PlayRobot(RobotMove):
    IMG = ROBOT
    START_POS = (random.uniform(40, 735), random.uniform(
        40, 535))  # start at random potition
    trail_set = []

    def collide(self):
        self.boundaryX = WIDTH - 64
        self.boundaryY = HEIGHT - 64
        # hit the wall
        if self.x <= 38:
            self.x = 38
        elif self.x >= self.boundaryX - 39:
            self.x = self.boundaryX - 39

        if self.y <= 38:
            self.y = 38
        elif self.y >= self.boundaryY - 39:
            self.y = self.boundaryY - 39

# Raycasting


def cast_rays(screen, walls):

    all_sensors = []

    sensor_x = player_robot.x+(ROBOT.get_width()/2)
    sensor_y = player_robot.y+(ROBOT.get_height()/2)

    temp_angle = 0
    for i in range(12):
        all_sensors.append((sensor_x, sensor_y, temp_angle, temp_angle, i))
        temp_angle += STEP_ANGLE

    for sensor in all_sensors:

        clip = None

        for depth in range(200):
            target_x = sensor[0] - math.sin(sensor[2]) * depth
            target_y = sensor[1] + math.cos(sensor[3]) * depth

            ray = ((sensor_x, sensor_y), (target_x, target_y))

            clipped_lineL = walls[0].clipline(ray)
            clipped_lineR = walls[1].clipline(ray)
            clipped_lineT = walls[2].clipline(ray)
            clipped_lineB = walls[3].clipline(ray)

            if clipped_lineR:
                clip = clipped_lineR[0]
            if clipped_lineL:
                clip = clipped_lineL[0]
            if clipped_lineT:
                clip = clipped_lineT[0]
            if clipped_lineB:
                clip = clipped_lineB[0]

        sensor_placement_offset = 8
        sensor_placement_radius_depth = 64
        sensor_placement_x = sensor[0] - math.sin(
            sensor[2]) * sensor_placement_radius_depth - sensor_placement_offset
        sensor_placement_y = sensor[1] + math.cos(
            sensor[3]) * sensor_placement_radius_depth - sensor_placement_offset
        collision_offset = 29
        if clip:
            sensor_distance = int(
                math.sqrt((clip[1]-sensor_y)**2 + (clip[0]-sensor_x)**2))-collision_offset
        else:
            sensor_distance = 200

        sensor_text = SENSORS_FONT.render(
            f"{sensor_distance}", 1, (255, 255, 255))
        screen.blit(
            sensor_text, (sensor_placement_x, sensor_placement_y))

    # ------------


# setting the enviroment

class Envir:
    def __init__(self, dimention):
        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        # map_dims
        self.height = dimention[0]
        self.width = dimention[1]
        # window setting
        self.map = pygame.display.set_mode((self.width, self.height))
        # trail
        self.trail_set = []
    # line route

    def trail(self, pos):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, self.white, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]))
        if self.trail_set.__sizeof__() > 10000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)
    # y and x axis

    def robot_frame(self, pos, rotation):
        n = 80
        centerx, centery = pos
        x_axis = (centerx + n*np.cos(-rotation), centery + n*np.sin(-rotation))
        y_axis = (centerx + n*np.cos(-rotation+np.pi/2),
                  centery + n*np.sin(-rotation+np.pi/2))
        pygame.draw.line(self.map, self.black, (centerx, centery), x_axis, 3)
        pygame.draw.line(self.map, self.black, (centerx, centery), y_axis, 3)

    def draw(self, screen, images, player_robot):

        # display images on screen
        for img, pos in images:
            screen.blit(img, pos)

        # display left, right velocity and theta on screen
        vel_text = MAIN_FONT.render(
            f"Vl = {player_robot.vl} Vr = {player_robot.vr} theta = {int(np.degrees(player_robot.theta))}", 1, self.white)
        screen.blit(vel_text, (10, HEIGHT - vel_text.get_height() - 40))

        # display robot on screen
        player_robot.draw(screen)
        # pygame.display.update()

    def setWalls():
        wall_pixel_offset = 42
        rectWallL = pygame.Rect(0, 0, wall_pixel_offset, HEIGHT)
        rectWallR = pygame.Rect(WIDTH-wall_pixel_offset, 0,
                                wall_pixel_offset, HEIGHT)
        rectWallT = pygame.Rect(0,  0, WIDTH, wall_pixel_offset)
        rectWallB = pygame.Rect(0, HEIGHT-wall_pixel_offset,
                                WIDTH, wall_pixel_offset)
        return [rectWallL, rectWallR, rectWallT, rectWallB]


# running game or not
run = True

images = [(BACKGROUND, (0, 0)), (WALL, (342, 142)), (WALL, (342, 342))]
# the robot
player_robot = PlayRobot()

# enviroment prints
enviroment = Envir([600, 800])
walls = Envir.setWalls()


# dt
dt = 50
clock = pygame.time.Clock()
FPS = 60
scanner_cooldown = 0

# simulation loop
while run:

    # activate quit button
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    # timer
    clock.tick(FPS)

    # activate buttons
    keys = pygame.key.get_pressed()
    key = [keys[pygame.K_w], keys[pygame.K_s], keys[pygame.K_o], keys[pygame.K_l],
           keys[pygame.K_t], keys[pygame.K_g], keys[pygame.K_x]]

    # run the robot
    activate = player_robot.move(key, dt)
    player_robot.collide()

# visualize objects
    enviroment.draw(SCREEN, images, player_robot)
    enviroment.robot_frame(
        (player_robot.x, player_robot.y), player_robot.theta)
    enviroment.trail((player_robot.x, player_robot.y))
    player_robot.draw(enviroment.map)
    cast_rays(SCREEN, walls)

    # ---

    pygame.display.update()

# exit the game
pygame.quit()
