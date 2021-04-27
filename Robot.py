# -*- coding: utf-8 -*-
"""
Date: 2021

@author: Michael Gibbs
"""

"""
    ROBOT MODULE
"""
import pygame
import numpy as np
import random
import math

"""
Robot Class

@functions:
    __init__ - This is the constructor of the Robot class.
    draw - This will draw the robot to the screen.
    move - This will generate a movement based upon the observations.
    motionUpdate - A function to return a prediction of mu and sigma
    perceptionUpdate - A function to update mu and sigma using the predictions
            from the motion update.
    observe - This will use the landmarks around to give a range and bearing
            to each in sight.
@attributes:
    POINTS_IN_WALL - This represents the number of points within a wall.
    N - This is the number of landmarks excluding walls.
    RANGE - This is the number of pixels the robot can see in front.
    mu - This represents the vector mu in the EKF-SLAM algorithm.
    sigma - This represents the matrix sigma in the EKF-SLAM algorithm.
    MAP_WIDTH - This is the map width.
    MAP_HEIGHT - This is the map height.
    Qt - This is a matrix representing the sensor noise.
    Rt - This is a matrix representing the motion noise.
    x - This is a 1x3 state vector representing the true coordinates and 
    bearing of the robot.
    FOV - This is half the total field of view, that is, it is the bearing in
    each direction from the robots bearing.

"""

class Robot:
    
    POINTS_IN_WALL = 15
    N = 80
    
    FOV = np.pi/3
    
    RANGE = 200
    
    mu = []
    sigma = []
    MAP_WIDTH = 0
    MAP_HEIGHT = 0
    
    # Line 6
    # Sensor Noise
    Qt = np.matrix([[0.01, 0, 0],[0, 0.01, 0], [0, 0, 0.01]])
    # Motion Noise
    Rt = 0.5 *np.array([[0.1,0,0],
               [0,0.01,0],
               [0,0,0.01]])

    """
    @params:
        Walls - This denotes the number of walls (Integer)
        w - This denotes the width of the map (Integer)
        h - This denotes the height of the map (Integer)
    This is the constructor for the robot class. It will initialise all of the
    required values for the SLAM algorithm.
    """
    def __init__(self, Walls, w, h):
        print("Hi, I am a new Robot!")
        self.x = [100, 75, -np.pi] 
        self.N = self.N + Walls*self.POINTS_IN_WALL
        self.MAP_WIDTH = w
        self.MAP_HEIGHT = h
        self.WALLS = Walls
        self.mu = np.zeros((3*self.N + 3)) 
        self.mu[:3] = self.x
        self.sigma = 1e6*np.eye(3*self.N + 3)
        self.sigma[:3,:3] = np.zeros((3,3))
    
    """
    @params:
        surface - This is the surface that the drawing will draw to 
        (Pygame Surface)
    This will draw the robot to the surface passed in.
    """
    def draw(self, surface):

        pygame.draw.circle(surface, color, (self.x[0], self.x[1]), ROBOT_RADIUS)
        pygame.draw.line(surface, color, (self.x[0], self.x[1]), (self.x[0] + self.RANGE*np.cos(self.x[2] + self.FOV), self.x[1] + self.RANGE*np.sin(self.x[2] + self.FOV)))
        pygame.draw.line(surface, color, (self.x[0], self.x[1]), (self.x[0] + self.RANGE*np.cos(self.x[2] - self.FOV), self.x[1] + self.RANGE*np.sin(self.x[2] - self.FOV)))
        
        
    """
    @params:
        Z - This is the set of observations ([Observations])
        
    @return:
        u - This represents a motion vector ([int, int, int])
        
    This will generate a movement vector (u) using the observations seen from 
    the "sensor".
    """
    def move(self, Z):
        """
            A Sufficient Movement Algorithm Must be Constructed
            
            Must stay within the boundaries
            Must avoid obstacles
        
        """
        
        movement = 3
        u_theta = 0

        ## IS GOING TO NEED A REFERENCE
        motion_noise = np.matmul(np.random.randn(1,3),self.Rt)[0]
        dtrans = movement + motion_noise[0]
        drot1 = u_theta + motion_noise[1]
        drot2 = 0 + motion_noise[2]
        #[dtrans, drot1, drot2] = u[:3] #+ motion_noise
        
        x = self.x
        x_new = x[0] + dtrans*np.cos(x[2]+drot1)
        y_new = x[1] + dtrans*np.sin(x[2]+drot1)
        
        # This marks the edges of the screen.
        if x_new <= 15 and y_new <= 15:
            u_theta = np.pi
        elif x_new < 15 or x_new > self.MAP_WIDTH-15:
            # Need to rotate and check.
            angle = np.tan(self.x[0]/self.x[1])
            movement = 1
            u_theta = np.pi - 2*angle
            x_new = x[0]
            y_new = x[1]  
        elif y_new < 15 or y_new > self.MAP_HEIGHT-15:
            angle = np.tan(self.x[1]/self.x[0])
            movement = 1
            u_theta = np.pi - 2*angle
            x_new = x[0]
            y_new = x[1]
          
        # This will check if a landmark is within range.
        for [Range, Bearing, correspondence, s] in Z:
            if Range <= ROBOT_RADIUS + 5 and abs(Bearing) < np.pi/6:
                u_theta = np.pi - 2*Bearing
                break
                
        # Make sure the bearings are within the range -pi <= theta <= pi.
        theta_new = (x[2] + u_theta + drot2 + np.pi) % (2*np.pi) - np.pi
        
        # Update the robots x position
        self.x = [x_new, y_new, theta_new]
        
        return [movement, u_theta, 0]
        
    """
    @params:
        mu - This holds all of the landmark positions and robot position
        sigma - The covariance matrix
        u - A control vector
    @return:
        muBar - estimate of mu after motion
        sigmaBar - estimate of sigma after motion
    """        
    def motionUpdate(self, mu, sigma, u):
        """
        Title: EKF-SLAM
        Author: Attila94
        Date: 7/12/2018
        Version: Unknown
        Available at: https://github.com/Attila94/EKF-SLAM
        """
        [dtrans, drot1, drot2] = u
        
        # Line 2
        # Matrix that is used to map the motion model to the dimension of mu.
        mapF = np.append(np.eye(3),np.zeros((3,3*self.N)),axis=1)
        # Line 3
        # Matrix multiplication to map this motion model to the dimension of mu.
        muBar = mu + mapF.T.dot(np.array([[dtrans*np.cos(mu[2] + drot1)],
                                         [dtrans*np.sin(mu[2] + drot1)],
                                         [drot1 + drot2]])).T
        muBar = muBar.squeeze()
        muBar[2] = (muBar[2] + np.pi) % (2*np.pi) - np.pi
        
        # Line 4
        # Define a jacobian to linearise the motion.
        Jacobian = np.matrix([[0,0,-dtrans*np.sin(mu[2]+drot1)],
                   [0,0,dtrans*np.cos(mu[2]+drot1)],
                   [0,0,0]])
        
        # This will estimate the error
        Gt = np.identity(len(mu)) + mapF.T.dot(Jacobian).dot(mapF)
        
        
        # Line 5
        # Gain an estimate for sigma
        SigmaBar = Gt.dot(sigma.dot(Gt.T)) + mapF.T.dot(self.Rt.dot(mapF))
        
        print("Predicted: X:" + str(muBar[0]) + "\tY: " + str(muBar[1]) + "\tTheta: " + str(muBar[2]))
        return muBar, SigmaBar
        
    """
    @params:
        mu - This is a vector containing the estimate of the robot position 
            and the landmark positions.
        sigma - This contains the covariance of all landmarks against each 
            other.
        observation - A set of vectors of the observations.
    @return:
        mu - This is a vector containing the estimate of the robot position 
            and the landmark positions.
        sigma - This contains the covariance of all landmarks against each 
            other.
        
    This perception update will give an estimation of mu and sigma.
    """    
    def perceptionUpdate(self, mu, sigma, observation):
        
        # Line 7
        for [Range, Bearing, s, correspondence] in observation:
            #print(correspondence)
            
            # Landmark j
            j = 3*correspondence + 3
            # Line 9, i.e. Landmark has never been seen.
            if sigma[j, j] >= 1e6 and sigma[j+1, j+1] >= 1e6:
                # Line 10
                # Update the range, bearing and signature    
                mu[j] = mu[0] + Range * np.cos(mu[2] + Bearing)
                mu[j+1] = mu[1] + Range * np.sin(mu[2] + Bearing)
                mu[j+2] = s
            
            # Line 12
            # Gain an estimate for the x and y.
            deltax = mu[j] - mu[0]
            deltay = mu[j + 1] - mu[1]
            delta = np.array([deltax, deltay]).T
            
            # Line 13
            # Distance squared
            q = delta.T.dot(delta)
            
            
            # Line 14
            # Estimate the relative angle.
            angle = (math.atan2(deltay, deltax))
            # Collect the estimation data.
            zHat = np.array([np.sqrt(q), math.atan2(deltay, deltax) - mu[2], s]).T
            
            # To construct fMap, one needs to construct blocks
            A = np.identity(3)
            B = np.zeros((3,3))
            C = np.zeros((3, 3*(correspondence+1)-3))
            D = np.zeros((3, 3*self.N - 3*(correspondence+1)))
            
            fMap = np.block([[A, C, B, D],
                             [B, C, A, D]])
            
                        
            """
            The below code is adapted from this source for the purposes of...
            
            Title: EKF-SLAM
            Author: Attila94
            Date: 7/12/2018
            Version: Unknown
            Available at: https://github.com/Attila94/EKF-SLAM
            """
            sq = np.sqrt(q)
            # This is the portion of code referenced from above.
            Jacobian = np.array([[-sq*deltax, -sq*deltay, 0, sq*deltax, sq*deltay, 0],
                            [deltay, -deltax, -q, -deltay, deltax, 0],
                            [0, 0, 0, 0, 0, 1]], dtype='float')
            
            # Avoid Division by Zero
            if (q == 0): H = np.zeros((Jacobian.shape[0], fMap.shape[1]))
            else: H = (1/q) * (Jacobian.dot(fMap))
            
            # Kalman Gain
            K = sigma.dot(H.T).dot(np.linalg.inv(H.dot(sigma).dot(H.T) + self.Qt))
            
            # Difference in the expected vs the actual
            absZ = ((np.matrix((Range - zHat[0], Bearing - zHat[1], s - zHat[2])) +  np.pi) % (2*np.pi) - np.pi).T
            a = K.dot(absZ)
            A = np.squeeze(np.asarray(a))
            # Update mu and sigma
            mu = mu + A
            sigma = (np.identity(3*self.N + 3) - K.dot(H)).dot(sigma)
        return mu, sigma
    
    """
    @params:
        points_list - A list of points/walls in the drawing list.
    @returns:
        observations - All landmarks that are visible to the robot.
    
    This function will work out which landmarks are within the observable 
    range and then convert their positions into observations.
    """
    def Observe(self, points_list):
        
        # List of observed points
        observations = []
        
        # Initialise the number of walls present.
        wallNo = 0
        
        for point in points_list:
            if not point.isWall:
                # This works out the angle to the origin.
                angle = (math.atan2((point.y-self.x[1]),(point.x-self.x[0])))
                left_angle = ((self.x[2] + 2*np.pi + self.FOV) % (2*np.pi) - angle + np.pi) % (2*np.pi) - np.pi
                right_angle = ((self.x[2] + 2*np.pi - self.FOV) % (2*np.pi) - angle + np.pi) % (2*np.pi) - np.pi 
                # In field of view
                if left_angle > 0 and right_angle < 0:
                    # Range
                    point_Range = np.sqrt((point.x - self.x[0])**2 + (point.y - self.x[1])**2)
                    if point_Range <= self.RANGE:
                        # Relative angle to the robot
                        relativeAngle = (angle - self.x[2] + np.pi) %(2*np.pi) -np.pi
                        correspondence = points_list.index(point)
                        
                        observation = [point_Range, relativeAngle, 0, correspondence]
                        observations.append(observation)
                
            # Observe Wall Points
            else: 
                wallNo += 1
                for wallPoint in point.WallPoints:
                    # This works out the angle to the origin
                    angle = (math.atan2((wallPoint.y-self.x[1]),(wallPoint.x-self.x[0])))
                    left_angle = ((self.x[2] + 2*np.pi + self.FOV) % (2*np.pi) - angle + np.pi) % (2*np.pi) - np.pi
                    right_angle = ((self.x[2] + 2*np.pi - self.FOV) % (2*np.pi) - angle + np.pi) % (2*np.pi) - np.pi 
                    if left_angle > 0 and right_angle < 0:
                        # Range
                        point_Range = np.sqrt((wallPoint.x - self.x[0])**2 + (wallPoint.y - self.x[1])**2)
                        if point_Range <= self.RANGE:
                            # Relative angle to the robot
                            relativeAngle = (angle - self.x[2] + np.pi) %(2*np.pi) -np.pi
                            correspondence = len(points_list)-self.WALLS + point.WallPoints.index(wallPoint) + (wallNo-1)*self.POINTS_IN_WALL
                            
                            observation = [point_Range, relativeAngle, wallNo, correspondence]
                            observations.append(observation)
        return observations
        
    
ROBOT_RADIUS = 10
color = (128, 128, 128)