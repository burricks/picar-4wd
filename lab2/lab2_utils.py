import picar_4wd as fc

def isAnythingAhead():
    aheadDistance = fc.get_distance_at(0)
    if aheadDistance == -2 or aheadDistance > 100:
        return False
    return True

def goForward():
    fc.forward(20)

def goBackwards():
    fc.backward(20)

def goRight():
    fc.goRight(20)

def goLeft():
    fc.goLeft(20)

def stop():
    fc.stop()