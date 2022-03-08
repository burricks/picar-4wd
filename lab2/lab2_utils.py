import picar_4wd as fc
import json

fc.start_speed_thread()


class Stats():
    def __init__(self, speed, distanceAhead):
        self.speed = speed
        self.distanceAhead = distanceAhead

    def toDict(self):
        return json.dumps({"speed": self.speed, "distanceAhead": self.distanceAhead})


def distanceAhead():
    aheadDistance = fc.get_distance_at(0)
    if aheadDistance == -2 or aheadDistance > 100:
        return ">100"
    return str(aheadDistance)

def speed():
    return fc.speed_val()

def getStats():
    return Stats(speed(), distanceAhead()).toDict()

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
    fc.turn_right(20)

def goLeft():
    fc.turn_left(20)

def stop():
    fc.stop()