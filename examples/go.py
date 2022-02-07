import picar_4wd as fc
import math
import sys
import numpy as np
from advanced_map import main as adv_map
from picar_4wd.speed import Speed
import time

ADDITIONAL_DEGREES = 40
DEGREES_TO_SCAN = 180

CURRENT_X = 0
CURRENT_Y = 0

def main():
    args = sys.argv[1:]
    xDest = int(args[0])
    yDest = int(args[1])
    if not atDestination(xDest, yDest):
        print("not at destination yet")
        #print(getAngleToDest(xDest, yDest))
        #scanPathToDest()
        navigateToDestination()
    else:
        print("at destination")
    #while not at destination:
        #navigateToDestination(degreesToScan

def atDestination(xDest, yDest):
    return (CURRENT_X, CURRENT_Y)== (xDest, yDest)


def navigateToDestination():
    moveForwardDistance = getMoveForwardDistance()
    if moveForwardDistance > 10:
        delta = moveForward(moveForwardDistance)
        #print(delta)

    """ else:
        if DestinationIsToTheRight() and iCanMoveRight():
            delta = moveRight()
        else if DestinationIsToTheLeft() and iCanMoveLeft():
            delta = moveLeft()
        else if DestinationIsStraightAhead():
            if iCanMoveRight():
                delta = moveRight()
            else if iCanMoveLeft():
                delta = moveLeft()
            else:
                delta = moveBackwards()
        else:
            raise Exception ("destination is behind you?")
    updateCurrentLocation(delta) """

def getMoveForwardDistance():
    dist5 = max(fc.get_distance_at(5),0)
    dist0 = max(fc.get_distance_at(0),0)
    distn5 = max(fc.get_distance_at(-5),0)
    print(dist0)
    #print(dist5)
    #print(distn5)
    #return myround(min(dist0,dist5, distn5))
    return myround(dist0)

#https://stackoverflow.com/questions/2272149/round-to-5-or-other-number-in-python
def myround(x, base=2):
    return base * round(x/base)

def moveForward(moveForwardDistance):
    distanceTraveled = 0
    speed4 = Speed(25)
    speed4.start()
    fc.forward(25)
    while distanceTraveled < moveForwardDistance - 8:
        time.sleep(0.1)
        distanceTraveled += speed4() * .1
        print(distanceTraveled)
    fc.stop()
    speed4.deinit()
    return (0, moveForwardDistance)


    #updateCurrentLocation()
    #angle = getAngleToDest()
    #scanPathToDest(angle, degreesToScan)

def getAngleToDest(xDest, yDest):
    if yDest == 0:
        if xDest > 0:
            return 90
        return -90
    return np.degrees(np.arctan((xDest/yDest)))

#def scanPathToDest():
    #local_map = adv_map()
    #max_x_scanned = CURRENT_X + 100
    #min_x_scanned = CURRENT_X - 100
    #max_y_scanned = CURRENT_Y + 100
    #print(map)
    #scan surroundings
    #go to nearest spot on scanned map

#def getNextDestination():
    #the best spot on the scanned map that the car can currently get to


    #scan degreesToScan
    #if no obstacle detected:
        #proceed to max distance in angle direction
        #update current location
    #else if degreesToScan == 180:
        #go to the closest possible spot on map
    #else
        #scanPathToDest(angle, degreesToScan + additionalDegrees)

if __name__ == '__main__':
    main()

