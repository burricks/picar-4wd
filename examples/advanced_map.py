import picar_4wd as fc
import math
import numpy as np

def main():
    map = [[0 for i in range(-100,101)] for j in range(100)]
    n = 1
    for angle in range(-90, 91):
        distanceToObject = fc.get_distance_at(angle)
        if distanceToObject > 0 and distanceToObject < 100:
            (x, y) = getArrayIndices(angle, distanceToObject)
            map[y][x] = n
            #n+=1
        #if distanceToObject == -2:
            #(x, y) = getArrayIndices(angle, distanceToObject)
            #map[y][x] = -2
    printResult(map)

def printResult(map):
    for j in range(0,len(map)):
        for i in range(0, len(map[j])):
            print(map[j][i], end="")
        print("")

def adjustX(orig_angle, x):
    if orig_angle < 0:
        return 100 - int(x)
    return 100 + int(x)

def getArrayIndices(angle, distance):
    orig_angle = angle
    if (angle == 90):
        x = distance
        y = 0
    if (angle == 0):
        x = 0
        y = distance
    if (angle == -90):
        x = -distance
        y = 0
    else:
        angle = 90 - abs(orig_angle)
        #check inputs. Angle bounds? Distance bounds? 
        x = math.cos(np.radians(angle)) * distance
        y = math.sin(np.radians(angle)) * distance
    print("original angle:" + str(orig_angle))
    print("angle: " + str(angle))
    print("distance: " + str(distance))
    print("coordinates" + str(adjustX(orig_angle, x)) + "," + str(int(y)))
    return (adjustX(orig_angle, x),int(y))

if __name__ == '__main__':
    main()
