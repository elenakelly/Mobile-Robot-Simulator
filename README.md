# Mobile Robot Simulator

Within the program you are allowed to move the robot in a closed room using the buttons of the PC’s keyboard.<br />
When the robot collides with wall, moves the robot realistically along side of walls without entering the wall.<br />
In the Screen:
* Displaying values of distance sensors as plain numbers
* Displaying motor speed and angle values as plain numbers

## Motion Model
The motion model is a wheeled robot with differential drive<br />
<img width="554" alt="Screenshot 2022-03-22 at 5 54 03 PM" src="https://user-images.githubusercontent.com/59971317/159538001-52a6ea7f-fa25-44dc-8ae0-9737b2585b90.png">


Control speed of motors of left and right wheel with keyboard<br />
* W: positive increment of left wheel motor speed<br />
* S: negative increment of left wheel motor speed<br />
* O: positive increment of right wheel motor speed<br />
* L: negative increment of right wheel motor speed<br />
* T: positive increment of both wheels’ motor speed<br />
* G: negative increment of both wheels’ motor speed<br />
* X: both motor speeds are zero<br />

## Sensor Model
Details for the sensor model:<br />
* Implemented 12 sensors<br />
* 30° distance for each sensors<br />
* Model sensors are visualized as straight lines<br />
* Intersections with lines marking walls <br />
* Limited distance measure<br />
* Used software library for calculations<br />

## Collision Handling
Details for the collision:<br />
* fixed time step to allow the robot to move through wall at high speeds with:<br />
    * backwards check<br />
    * when collision occurred calculates intermediate time steps<br />
* When the driver moves into the wall, calculations of velocity components perpendicular and parallel to wall
* Only velocity parallel to wall contributes to actual motion




## Installation
The program is in Python <br />
In order to use the code you need to install Numpy, Math and Pygame packages
   ```sh
    import numpy as np
    import pygame
    import math
   ```

## Contributors
Elena Kane </br>
Nikolaos Ntantis </br>
Ioannis Montesantos 
