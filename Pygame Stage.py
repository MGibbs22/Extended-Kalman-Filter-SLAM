# -*- coding: utf-8 -*-
"""
Date: 2021

@author: Michael Gibbs
"""

import random
import pygame
import numpy as np
from Robot import Robot
from Testing import Testing

"""
Wall Class

This class is responsible for the walls within the environment.

@attributes:
    start - A start point of the line (Point).
    end - An end point of the line (Point).
    wallPoints - A list of points that are between the start and end ([Point]).
    isWall - A value used for the differentiation between a wall and a point
            (Boolean).
@funcctions:
    __init__: The constructor
    drawMe: A function used to draw the Wall onto the screen.
"""

class Wall:
    """
    @params:
        x1 - The x coordinate of the start point (Integer).
        y1 - The y coordinate of the start point (Integer).
        x2 - The x coordinate of the end point (Integer).
        y2 - The y coordinate of the end point (Integer).
    This is the constructor of the class. It will initialise the attributes 
    either randomly or with the passed in parameters.
    """
    def __init__(self, x1 = None, y1 = None, x2 = None, y2 = None):
        if x1 != None:
            self.start = point(x1, y1)
            self.end = point(x2, y2)
            
        else:
            self.start = point()
            self.end = point()
            
        self.WallPoints = []
        self.isWall = True
          
        
    """
    This function is used to draw the wall to the screen.
    
    """   
    def drawMe(self):
        """ 
      for i in range(0, len(self.WallPoints)-2, 2):
            pygame.draw.line(screen, color, (self.WallPoints[i].x, self.WallPoints[i].y), (self.WallPoints[i+1].x, self.WallPoints[i+1].y), 5)
            pygame.draw.line(screen, color, (self.WallPoints[i+1].x, self.WallPoints[i+1].y), (self.WallPoints[i+2].x, self.WallPoints[i+2].y), 5)
            """
        pygame.draw.line(screen, color, (self.start.x, self.start.y), (self.end.x, self.end.y), 5)
    
    """
    This function is used to generate all point between the walls start and 
    end points.
    """
    def addPoints(self):
        dx = (self.end.x - self.start.x) / float(POINTS_IN_WALL)
        dy = (self.end.y - self.start.y) / float(POINTS_IN_WALL)
        for i in range(0, POINTS_IN_WALL):
            newPoint = point(self.start.x + i * dx, self.start.y + i * dy)
            self.WallPoints.append(newPoint)
            
        print(str(len(self.WallPoints)))
    
"""
Point Class
         
This class is used to associate x and y coordinates into its own type.

@functions:
    __init__ - This is the constructor of the point class.
    drawMe - This is the function used to draw the point onto the screen.
@attributes:
    x - This is the x coordinate of the point (Integer).
    y - This is the y coordinate of the point (Integer).
    isWall - This is a value used to differentiate between a wall and a point
            (Boolean).

"""    
class point:

    """
    @params:
         x - This is the x coordinate of the point (Integer).
         y - This is the y coordinate of the point (Integer).
    This function is the constructor of the point class. Should no parameters 
    be given, a random point will be generated.
    """
    def __init__(self, x = None, y = None):
        
        if x == None:
            self.x = random.randint(CROSS_SIZE, MAP_WIDTH - CROSS_SIZE)
            self.y = random.randint(CROSS_SIZE, MAP_HEIGHT - CROSS_SIZE)
        else:
            self.x = x
            self.y = y
        self.isWall = False
                
    """
    This function will draw the point to the screen.
    """
    def drawMe(self):
        pygame.draw.line(screen, color, (self.x-CROSS_SIZE, self.y-CROSS_SIZE), (self.x+CROSS_SIZE, self.y+CROSS_SIZE), 2)
        pygame.draw.line(screen, color, (self.x+CROSS_SIZE, self.y-CROSS_SIZE), (self.x-CROSS_SIZE, self.y+CROSS_SIZE), 2)
        
    
"""
Drawing List

This classes main function is to have a list containing many types of which 
one could call the draw functions.

@functions:
    __init__ - This is the constructor of the class.
    appendPoint - This will append an object to the points list.
    draw - This will call the draw functions of the objects within the drawing
            list.
@attributes:
    points_List - This holds the points and/or walls in a list.
"""
class drawingList:
    points_List = []
    """
    This is simply an empty constructor.
    """
    def __init__(self):
        print("New Drawing List")
    
    """
    @params: 
        point1 - This is an object that will be appended to the points_List.
                This item is polymorphic and does not have an explicit type.
                Although the object must have a member called draw().
    This function will append the object point1 to the points_List.
    """
    def appendPoint(self, point1):
        self.points_List.append(point1)
        
    """
    This function will call the draw functions of all the objects within the
    points_List.
    """
    def draw(self):
        for point in self.points_List:
            point.drawMe()
            
        #for point in self.copy_List:
         #   point.draw_cross()

Test = Testing()

# Denotes the number of points that will be in the wall
POINTS_IN_WALL = 15

# Map dimensions
MAP_WIDTH = 350
MAP_HEIGHT = 250

CROSS_SIZE = 4

NUM_OF_WALLS = 2

# Create an instance of the Robot, Clock and drawingList
robot = Robot(NUM_OF_WALLS, MAP_WIDTH, MAP_HEIGHT)
clock = pygame.time.Clock()
points_List = drawingList()

# Initialise the Pygame screen
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption('EKF-SLAM')

# Another layer to draw over
image = pygame.Surface([800,600], pygame.SRCALPHA, 32)
image = image.convert_alpha()

color = (0, 0, 0)
done = False

# This will generate points.
NUM_OF_POINTS = robot.N - NUM_OF_WALLS*POINTS_IN_WALL
for i in range(0, NUM_OF_POINTS):
    newPoint = point()
    points_List.appendPoint(newPoint)

"""
These are used for testing purposes.
"""
#wall1 = Wall(30, 30, 30, 200) 
#wall2 = Wall(30, 200, 200, 200)
#wall3 = Wall(30, 30, 200, 30)
#wall4 = Wall(200, 30, 200, 200)
#wall1.addPoints()
#points_List.appendPoint(wall1)
#wall2.addPoints()
#points_List.appendPoint(wall2)
#wall3.addPoints()
#points_List.appendPoint(wall3)
#wall4.addPoints()
#points_List.appendPoint(wall4)

# This will create new walls.
for i in range(0, NUM_OF_WALLS):
    newWall = Wall()
    newWall.addPoints()
    points_List.appendPoint(newWall)

# A tuple of goal coordinates
GOAL = (200, 220)

# Initialise a time step
t = 0
while not done:
    t += 1
    """
        AIM: To gain the obstacles within this cone and add a correspondence.
        This will make looking into the algorithm itself easier and the 
        algorithm more applicable overall.

    """    
      
    # The correspondence will be that of the INDEX of in the list.
    Observations = robot.Observe(points_List.points_List)
    
    # Get a new control unit and move.
    u = robot.move(Observations)
    
    # Estimate using the motion model.
    muBar, sigmaBar = robot.motionUpdate(robot.mu, robot.sigma, u)
    
    # Make a final estimation using the prior estimates and the perception
    # update.
    robot.mu, robot.sigma = robot.perceptionUpdate(muBar, sigmaBar, Observations)
    
    # Tests
    if robot.sigma.T.all() != robot.sigma.all():
        print("Oh No")
        
    if (robot.mu[0] > 800 or robot.mu[0] < 0):
        print("Oh No X")
        
    if (robot.mu[1] > 600 or robot.mu[1] < 0):
        print("Y Oh No")
        
    if (robot.mu[2] > np.pi or robot.mu[2] < -np.pi):
        print("Angles")
        
    error = abs(robot.x - robot.mu[0:3])
    
    # Reset the screen
    screen.fill((255, 255, 255))
    
    # Draw the robot and points onto the screen
    robot.draw(screen)
    points_List.draw()
    
    
    """
    This section of code will draw the wall and point estimates.
    """
    pygame.draw.circle(screen, (255,0,0), (robot.mu[0] + MAP_WIDTH, robot.mu[1] + MAP_HEIGHT), 10)
    for i in range(3, len(robot.mu)-3, 3):
        if robot.mu[i] != 0 and robot.mu[i + 1] != 0 and robot.mu[i + 3] != 0 and robot.mu[i + 4] != 0:
            if robot.mu[i+2] == robot.mu[i+5] and robot.mu[i+2] != 0:
                pygame.draw.line(screen, (255,0,0), (robot.mu[i] + MAP_WIDTH, robot.mu[i+1] + MAP_HEIGHT), (robot.mu[i + 3] + MAP_WIDTH, robot.mu[i+4] + MAP_HEIGHT), 2)
                #pygame.draw.circle(screen, (255,255,0), (robot.mu[i]-CROSS_SIZE + MAP_WIDTH, robot.mu[i+1]-CROSS_SIZE + MAP_HEIGHT), 5)
    
        if robot.mu[i+2] == 0 and robot.mu[i] != 0 and robot.mu[i + 1] != 0:    
            pygame.draw.line(screen, (255,0,0), (robot.mu[i]-CROSS_SIZE + MAP_WIDTH, robot.mu[i+1]-CROSS_SIZE + MAP_HEIGHT), (robot.mu[i]+CROSS_SIZE + MAP_WIDTH, robot.mu[i+1]+CROSS_SIZE + MAP_HEIGHT), 2)
            pygame.draw.line(screen, (255,0,0), (robot.mu[i]+CROSS_SIZE + MAP_WIDTH, robot.mu[i+1]-CROSS_SIZE + MAP_HEIGHT), (robot.mu[i]-CROSS_SIZE + MAP_WIDTH, robot.mu[i+1]+CROSS_SIZE+ MAP_HEIGHT), 2)
            #clock.tick(60)
    if robot.mu[-1] == 0 and robot.mu[-2] != 0 and robot.mu[-3] != 0:
        pygame.draw.line(screen, (255,0,0), (robot.mu[-3]-CROSS_SIZE + MAP_WIDTH, robot.mu[-2]-CROSS_SIZE + MAP_HEIGHT), (robot.mu[-3]+CROSS_SIZE + MAP_WIDTH, robot.mu[-2]+CROSS_SIZE + MAP_HEIGHT), 2)
        pygame.draw.line(screen, (255,0,0), (robot.mu[-3]+CROSS_SIZE + MAP_WIDTH, robot.mu[-2]-CROSS_SIZE + MAP_HEIGHT), (robot.mu[-3]-CROSS_SIZE + MAP_WIDTH, robot.mu[-2]+CROSS_SIZE+ MAP_HEIGHT), 2)
       
    # Simple check if the robot is in the goal state.
    """
    goalDistance = (np.sqrt((robot.x[0] - GOAL[0])**2 + (robot.x[1] - GOAL[1])**2))
    if goalDistance < 20:
        done = True
    else:
        pygame.draw.circle(screen, (0,255,0), GOAL, 20)
    """
    # If the window is exited break loop.
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True  
            
    # Join the layers
    screen.blit(image, ((0, 0), (800,600))) 
    
    pygame.display.flip()

    clock.tick(60)
    
    if t == 200: break
    
# Destroy the Window
#pygame.quit()
Test.stopTimer()
print(Test.getMAPE(points_List.points_List, robot.mu, robot.sigma))
