import picar_4wd as fc
import math
import sys
import numpy as np
from advanced_map import main as adv_map
from picar_4wd.speed import Speed
import time
from enum import IntEnum

ADDITIONAL_DEGREES = 40
DEGREES_TO_SCAN = 180

CURRENT_X = 0
CURRENT_Y = 0

X_DEST = 0
Y_DEST = 0

class Direction(IntEnum):
    LEFT = 2
    FORWARD = 1
    RIGHT = 0
    BACKWARD = 3

CURRENT_DIRECTION = Direction.FORWARD



def main():
    args = sys.argv[1:]
    global X_DEST
    global Y_DEST
    X_DEST = int(args[0])
    Y_DEST = int(args[1])
    print(getMoveForwardDistance())
    print(DestinationAheadDistance())
    while not atDestination():
        navigateToDestination()
    print("at destination, yay")
    #while not at destination:
        #navigateToDestination(degreesToScan

def atDestination():
    return abs(CURRENT_X - X_DEST) <= 18 and abs(CURRENT_Y - Y_DEST) <= 18

def navigateToDestination():
    moveForwardDistance = getMoveForwardDistance()
    destinationAheadDistance = DestinationAheadDistance()
    print("Destination Ahead Distance: " + str(destinationAheadDistance))
    print("Move Forward Distance: " + str(moveForwardDistance))
    delta = (0,0)
    if destinationAheadDistance > 18 and moveForwardDistance > 18:
        delta = moveForward(min(moveForwardDistance - 10, destinationAheadDistance - 10))
    else:
        if DestinationIsToTheRight(): # and getMoveDistanceAtAngle(-90) > 18:
            turnRight()
            #delta = moveForward(18)
        elif DestinationIsToTheLeft(): # and getMoveDistanceAtAngle(90) > 18:
            turnLeft()
            #delta = moveForward(18)
        elif DestinationIsStraightAhead():
            if getMoveDistanceAtAngle(-90) > 18:
                turnRight()
                delta = moveForward(18)
            elif getMoveDistanceAtAngle(90) > 18:
                turnLeft()
                delta = moveForward(18)
            else:
                reverse()
        else:
            raise Exception ("destination is behind you?")
    updateCurrentLocation(delta)   
    print(CURRENT_X, CURRENT_Y)

def printStatus():
    print("Current location: " + str(CURRENT_X) + "," + str(CURRENT_Y))
    print("Current direction: " + str(CURRENT_DIRECTION))

#positive destination ahead distance means that the destination is ahead
def DestinationAheadDistance():
    if CURRENT_DIRECTION == Direction.FORWARD:
        return Y_DEST - CURRENT_Y
    elif CURRENT_DIRECTION == Direction.BACKWARD:
        return CURRENT_Y - Y_DEST
    elif CURRENT_DIRECTION == Direction.LEFT:
        return CURRENT_X - X_DEST
    elif CURRENT_DIRECTION == Direction.RIGHT:
        return X_DEST - CURRENT_X
    else:
        raise Exception ("invalid direction")

def DestinationIsAhead():
    print("Destination is ahead")
    if CURRENT_DIRECTION == Direction.FORWARD:
        return CURRENT_Y < Y_DEST
    elif CURRENT_DIRECTION == Direction.BACKWARD:
        return Y_DEST < CURRENT_Y
    elif CURRENT_DIRECTION == Direction.LEFT:
        return X_DEST < CURRENT_X
    elif CURRENT_DIRECTION == Direction.RIGHT:
        return CURRENT_X < X_DEST

def DestinationIsToTheRight():
    print("Destination is to the right")
    if CURRENT_DIRECTION == Direction.FORWARD:
        return CURRENT_X < X_DEST
    elif CURRENT_DIRECTION == Direction.BACKWARD:
        return CURRENT_X > X_DEST
    elif CURRENT_DIRECTION == Direction.LEFT:
        return CURRENT_Y < Y_DEST
    elif CURRENT_DIRECTION == Direction.RIGHT:
        return CURRENT_Y > Y_DEST

def DestinationIsToTheLeft():
    print("Destination is to the left")
    return not DestinationIsToTheRight()

def DestinationIsStraightAhead():
    print("Destination is straight ahead")
    if CURRENT_DIRECTION == Direction.FORWARD or CURRENT_DIRECTION == Direction.BACKWARD: 
        return 10 >= abs(CURRENT_X - X_DEST)
    elif CURRENT_DIRECTION == Direction.RIGHT or CURRENT_DIRECTION == Direction.LEFT:
        return 10>= abs(CURRENT_Y - Y_DEST)

def getMoveForwardDistance():
    dist5 = min(fc.get_distance_at(5),100)
    dist0 = min(fc.get_distance_at(0),100)
    distn5 = min(fc.get_distance_at(-5),100)
    if min(dist0,dist5, distn5) == -2:
        return 100
    return myround(min(dist0,dist5, distn5))
    #return myround(dist0)

def getMoveDistanceAtAngle(angle):
    dist = fc.get_distance_at(angle)
    if dist == -2:
        return 100
    print("Get move distance at " + str(angle) + " is " + str(myround(dist)))
    return myround(dist)

#https://stackoverflow.com/questions/2272149/round-to-5-or-other-number-in-python
def myround(x, base=2):
    return base * round(x/base)

def moveForward(moveForwardDistance):
    distanceTraveled = 0
    speed4 = Speed(25)
    speed4.start()
    fc.forward(25)
    while distanceTraveled < moveForwardDistance - 8:
        print("moving")
        time.sleep(0.1)
        distanceTraveled += speed4() * .1
        print(distanceTraveled)
    fc.stop()
    speed4.deinit()
    return getPositionDelta(moveForwardDistance)

def getPositionDelta(distance):
    if CURRENT_DIRECTION == Direction.FORWARD:
        return (0, distance)
    elif CURRENT_DIRECTION == Direction.RIGHT:
        return (distance, 0)
    elif CURRENT_DIRECTION == Direction.LEFT:
        return (-distance, 0)
    elif CURRENT_DIRECTION == Direction.BACKWARD:
        return (0, -distance)

def turnRight():
    distanceTraveled = 0
    speed4 = Speed(25)
    speed4.start()
    fc.turn_right(25)
    while distanceTraveled < 13:
        time.sleep(0.1)
        distanceTraveled += speed4() * .1
    fc.stop()
    speed4.deinit()
    global CURRENT_DIRECTION
    CURRENT_DIRECTION = Direction((int(CURRENT_DIRECTION) - 1) % 4)
    print("Current Direction is " + str(CURRENT_DIRECTION))

def turnLeft():
    distanceTraveled = 0
    speed4 = Speed(25)
    speed4.start()
    fc.turn_left(25)
    while distanceTraveled < 13:
        time.sleep(0.1)
        distanceTraveled += speed4() * .1
    fc.stop()
    speed4.deinit()
    global CURRENT_DIRECTION
    CURRENT_DIRECTION = Direction((int(CURRENT_DIRECTION) + 1) % 4)
    print("Current Direction is " + str(CURRENT_DIRECTION))

def reverse():
    distanceTraveled = 0
    speed4 = Speed(25)
    speed4.start()
    fc.backward(25)
    while distanceTraveled < 15:
        time.sleep(0.1)
        distanceTraveled += speed4() * .1
        print(distanceTraveled)
    fc.stop()
    speed4.deinit()

def updateCurrentLocation(delta):
    dx, dy = delta
    global CURRENT_X
    global CURRENT_Y
    CURRENT_X += dx
    CURRENT_Y += dy
    print("Current location is " + str(CURRENT_X) + "," + str(CURRENT_Y))


    #updateCurrentLocation()
    #angle = getAngleToDest()
    #scanPathToDest(angle, degreesToScan)

def getAngleToDest(X_DEST, Y_DEST):
    if Y_DEST == 0:
        if X_DEST > 0:
            return 90
        return -90
    return np.degrees(np.arctan((X_DEST/Y_DEST)))

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

