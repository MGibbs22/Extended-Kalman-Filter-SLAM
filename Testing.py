# -*- coding: utf-8 -*-
"""
Date: 2021

@author: Michael Gibbs
"""

import numpy as np
import timeit

"""
Testing Class

@functions:
    __init__ - Will instantiate the class and start a timer.
    getMAPE - Will return the MAPE of a map compared to the estimated 
            positions.
    getDistance - Will return the distance between 2 points passed in.
    stopTimer - This will stop the timer set in the constructor.
"""
class Testing:
    def __init__(self):
        # Starts a timer upon instantiation.
        self.start = timeit.default_timer()
    
    """
    @params:
        points_List - The list of all landmarks on the field.
        mu - The landmark positions
        sigma - The covariance matrix
    @returns:
        MAPE - Mean Absolute Percentage Error
    
    This function will use a reference point to check the distances between 
    other objects. It will compare the estiamted to the actual and gain a MAPE 
    statistic.
    """
    def getMAPE(self, points_List, mu, sigma):
        # Initialisation of relevant variables
        referencePoint = None
        referencePointHat = None
        distances = []
        abserr = []
        # For all obstacles
        for points in points_List:
            # Check point is a wall type
            if not points.isWall:
                # If no reference point set current
                if referencePoint == None: referencePoint = points
                # Otherwise calculate distance to reference point
                else: distances.append(self.getDistance(referencePoint.x, referencePoint.y, points.x, points.y))
            
            # Implies Wall type
            else:
                # For every point in the wall
                for points2 in points.WallPoints:
                    # If no reference point set current
                    if referencePoint == None: referencePoint = points2
                    # Otherwise calculate distance to reference point
                    else: distances.append(self.getDistance(referencePoint.x, referencePoint.y, points2.x, points2.y))
        # For all the distances gathered
        for i in range(3, len(mu), 3):
            # Used to handle array index
            j = int(i/3)-2
            
            # If no reference point for estimate, set one
            if referencePointHat == None and mu[i] != 0 : referencePointHat = (mu[i], mu[i+1])
            # Check that the landmark has been seen
            elif mu[i] != 0 :
                # Get the estiamted difference and minus the actual
                estimatedDifference = self.getDistance(referencePointHat[0], referencePointHat[1], mu[i], mu[i+1])
                # Get the absolute error
                abserr.append(abs((distances[j] - estimatedDifference)/distances[j])*100)
        totErr = 0
        # Sum the absolute error
        for i in range(0, len(abserr)):
            totErr += abserr[i]
        
        # Return the Mean Absolute Percentage Error
        return 100 - totErr/len(abserr) 
        
    """
    @params:
        x1 - x coordinate of first point
        y1 - y coordinate of first point
        x2 - x coordinate of second point
        y2 - y coordinate of second point
    @returns:
        The difference between the 2 points.
        
    This will calculate the Euclidean distance between 2 points.
    """
    def getDistance(self, x1, y1, x2, y2):
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)
    
    """
    This function will stop the timer that is started when this object is 
    instantiated.
    """
    def stopTimer(self):
        stop = timeit.default_timer()
        totalTime = stop - self.start
        print('Time: ', totalTime)  




