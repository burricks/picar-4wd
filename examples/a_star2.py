from stringprep import map_table_b2
import picar_4wd as fc
import math
import sys
import numpy as np
#from advanced_map import main as adv_map
#from picar_4wd.speed import Speed
import time
from helps import PriorityQueue
from helps import Node
from helps import NodeMap
from helps import Direction
import move
import cv2


#destination node should be in multiples of 20
def main():
    currentDirection = Direction.FORWARD
    goalNode = Node(int(sys.argv[1]),int(sys.argv[2]))

    currentPos, goalNode, map = initializeEmptyMap(goalNode)
    print("Current pos: " + str(currentPos))
    print("goal node: "  + str(goalNode))

    setUpCamera()

    #3if True:
    while not atDestination(currentPos, goalNode):
        print("going again")
        map = scanSurroundings(currentPos, map)
        printResult(map)
        print(map[0])
        route = astar(currentPos, goalNode, heuristicFunction, map)
        print(*route)
        currentPos, currentDirection = followRouteToBorder(route, currentPos, currentDirection)

    print("yay, finally. Goodnight")
    #scan surroundings, mark obstacles as 1s 
    #compute route
    #follow route for 3 steps? 

def setUpCamera():


def atDestination(startNode, goalNode):
    return startNode.x == goalNode.x and startNode.y == goalNode.y

def printResult(map):
    for j in range(0,len(map)):
        for i in range(0, len(map[j])):
            print(map[j][i], end="")
        print("")

def followRouteToBorder(route, currentPos, currentDirection):
    yBorder = currentPos.y + 100 #why 100?
    xBorder = currentPos.x
    for i in range(len(route) - 1, -1, -1):
        print(i)
        print(route[i])
        if route[i].y < yBorder and abs(xBorder - route[i].x) < 100:
            currentPos, currentDirection = move.moveToDestination(currentPos, route[i], currentDirection)
    return currentPos, currentDirection

def initializeEmptyMap(destNode):
    heightOfMap = abs(destNode.y + 1)
    widthOfMap = abs(2 * destNode.x + 1)
    map = [[0 for i in range(widthOfMap)] for j in range(heightOfMap)] #https://stackoverflow.com/questions/13157961/2d-array-of-zeros
    currentPos = Node(destNode.x, 0)
    destNode = Node(destNode.x + destNode.x, destNode.y)
    return currentPos, destNode, map

def scanSurroundings(currentPosition, map):
    mapHeight = len(map)
    mapWidth = len(map[0])
    theLastOneWasAnObstacle = False
    LastObstacleNode = None
    LastObstacleDistance = None
    for angle in range(-90,90,5):
        distanceToObject = fc.get_distance_at(angle)
        time.sleep(.1)
        if distanceToObject > 0:
            (x, y) = getArrayIndices(angle, distanceToObject, currentPosition)
            #x = x + int(mapWidth/2)
            if theLastOneWasAnObstacle and abs(LastObstacleDistance - distanceToObject) <= 5:
                map = fillInOnes(LastObstacleNode, Node(x,y), map)
            if x < mapWidth and x > -1 and y > -1 and y < mapHeight:
                print(mapWidth)
                print(x)
                print(y)
                theLastOneWasAnObstacle = True
                LastObstacleNode = Node(x,y)
                LastObstacleDistance = distanceToObject
                map[y][x] = 1
            else:
                theLastOneWasAnObstacle = False
        else:
            theLastOneWasAnObstacle = False
    return map

def fillInOnes(LastNode, CurrNode, map):
    print("filling in ones")
    mapHeight = len(map)
    mapWidth = len(map[0])
    for x in range(min(LastNode.x, CurrNode.x), max(LastNode.x, CurrNode.x)):
        for y in range(min(LastNode.y, CurrNode.y), max(LastNode.y, CurrNode.y)):
            if x < mapWidth and y < mapHeight:
                map[y][x] = 1
    return map



def adjustX(orig_angle, x, currentPosition):
    if orig_angle < 0:
        return currentPosition.x + int(x)
    return currentPosition.x - int(x)

def getArrayIndices(angle, distance, currentPosition):
    print("getting Array iNdices")
    print(angle)
    print(distance)
    orig_angle = angle
    if (angle == 90):
        x = -distance
        y = 0
    if (angle == 0):
        x = 0
        y = distance
    if (angle == -90):
        x = distance
        y = 0
    else:
        angle = 90 - abs(orig_angle)
        #check inputs. Angle bounds? Distance bounds? 
        x = math.cos(np.radians(angle)) * distance
        y = math.sin(np.radians(angle)) * distance
    #print("original angle:" + str(orig_angle))
    #rint("angle: " + str(angle))
    #print("distance: " + str(distance))
    #print("coordinates" + str(adjustX(orig_angle, x)) + "," + str(int(y)))
    x = adjustX(orig_angle, x, currentPosition)
    y = int(y) + currentPosition.y
    print("(" + str(x) + "," + str(y) + ")")
    return (x,y)


    #getNeighbors(startNode)

def printNodeList(nodeList):
    for node in nodeList:
        print(str(node), sep=" ")

def getNeighbors(node, map):
    neighbors = []
    #print(str(len(map)))
    if node.y + 20 < len(map) and clearTo(node, 20, map, True):
        neighbors.append(Node(node.x, node.y + 20))
    if node.y - 20 > 0 and clearToDown(node, 20, map, True):
        neighbors.append(Node(node.x, node.y - 20))
    if node.x + 20 < len(map[0]) and clearTo(node, 20, map, False):
        neighbors.append(Node(node.x + 20, node.y))
    if node.x - 20 > 0 and clearToDown(node, 20, map, False):
        neighbors.append(Node(node.x - 20, node.y))  
    print(*neighbors)
    return neighbors

def clearTo(origNode, incr, map,y):
    if y:
        for i in range(origNode.y, origNode.y + incr):
            if map[i][origNode.x] == 1:
                return False
        return True
    if not y:
        for i in range(origNode.x, origNode.x + incr):
            if map[origNode.y][i] == 1:
                return False
        return True

def clearToDown(origNode, incr, map,y):
    if y:
        for i in range(origNode.y, origNode.y - incr - 1, -1):
            if map[i][origNode.x] == 1:
                return False
        return True
    if not y:
        for i in range(origNode.x, origNode.x - incr - 1, -1):
            if map[origNode.y][i] == 1:
                return False
        return True


def getNodesFromDistance(startingNode, distance, isRight):
    distance = 101 if distance == -2 else int(distance)
    nodes = []
    for x in range(20,distance, 20):
        x = x if isRight else -x
        nodes.append(Node(startingNode.x + x, startingNode.y))
    return nodes

def distanceAtXIsGreaterThanY(x, y):
    distanceAtX = fc.get_distance_at(x)
    return distanceAtX == -2 or distanceAtX > y

def getFrontNodes(startingNode):
    nodes = []

    #the (angle, distance) measurements needed to ensure the vehicle has a
    #clear path ahead at 20cm, 40cm, etc 
    l20 = [(27, 23), (0, 20), (-27, 23)]
    l40 = [(14, 42), (0, 40), (-14, 42)]
    l60 = [(10, 61), (0, 60), (-10, 61)]
    l80 = [(7, 81), (0,80), (-7, 81)]
    l100 = [(6, 101), (0,100), (-6, 101)]

    allMeasurements = [(20, l20), (40, l40), (60, l60), (80, l80), (100, l100)]

    for distance, measurementList in allMeasurements:
        clearPath = True
        for x, y in measurementList:
            if not distanceAtXIsGreaterThanY(x, y):
                clearPath = False
                return nodes

        if clearPath:
            nodes.append(Node(startingNode.x, startingNode.y + distance))

    return nodes


"""def getRightNodesFromRightDistance(startingNode, rightDistance):
    rightDistance = 101 if rightDistance == -2 else rightDistance
    rightNodes = []
    for x in range(20,rightDistance, 20):
        rightNodes.append(Node(startingNode.x + x, startingNode.y))

def getLeftNodesFromLeftDistance(startingNode, leftDistance):
    leftDistance = 101 if leftDistance == -2 else leftDistance
    leftNodes = []
    for x in range(20,leftDistance, 20):
         leftNodes.append(Node(startingNode.x - x, startingNode.y)) """



def astar(startNode, goalNode, heuristicFunction, map):
    #currentLocation = Node(0,0)
    #currentDirection = Direction.FORWARD

    nodeIndicesToExplore = []
    nodeMap = NodeMap()
    startNodeIndex = nodeMap.insert(startNode)
    nodeIndicesToExplore.append(startNodeIndex)

    cameFromDict = {}
    
    nodeMap.setGScore(0, startNodeIndex)
    nodeMap.setFScore(heuristicFunction(startNode, goalNode), startNodeIndex)

    while len(nodeIndicesToExplore) > 0:
        currNodeObject = nodeMap.getBestNodeObject(nodeIndicesToExplore) #might be getting the max distance here? 
        #nodeMap.printNodesAndFScores(nodeIndicesToExplore)
        #print("best Node: " + str(currNodeObject))
        nodeIndicesToExplore.remove(currNodeObject.index)
        #print("nodeIndicesToExplore: " + str(nodeIndicesToExplore))
        #currentLocation, currentDirection = move.moveToDestination(currentLocation, currNodeObject.node, currentDirection)
        if currNodeObject.node.x == goalNode.x and currNodeObject.node.y == goalNode.y: #how to determine if I am at the goal
            print("made it, yay")
            return constructPath(cameFromDict, currNodeObject.node)
        neighborNodes = getNeighbors(currNodeObject.node, map)
        for neighborNode in neighborNodes:
            neighborNodeIndex = nodeMap.insert(neighborNode)
            potentialGScore = currNodeObject.gScore + heuristicFunction(currNodeObject.node, neighborNode)
            if potentialGScore < nodeMap.getNodeObject(neighborNodeIndex).gScore:
                cameFromDict[neighborNode] = currNodeObject.node
                nodeMap.setGScore(potentialGScore, neighborNodeIndex)
                nodeMap.setFScore(potentialGScore + heuristicFunction(neighborNode, goalNode), neighborNodeIndex)
                #print("nodeIndicesToExplore: " + str(nodeIndicesToExplore))
                #print("neighborNodeIndex: " + str(neighborNodeIndex))
                if not neighborNodeIndex in nodeIndicesToExplore:
                    nodeIndicesToExplore.append(neighborNodeIndex)
                    #print("nodeIndicesToExplore: " + str(nodeIndicesToExplore))

        #SCAN to get all neighbors that I can go to. I need an array of x and y coordinate sets that I can go to next
        #Add all neighbors to the nodemap
        #for each neighbor
            #tentative g score is the current g score plus the distance from current to neighbor
            #if the tentative g score is less than the current g score
                #nodeMap.setCameFromNode(currNode, neighborIndex)
                #nodeMap.setGScore(tentativeGScoree, neighborIndeex)
                #nodeMap.setFScore(tentativeGScore + heuristicFunction(neighborNode, endNode), neighborIndex)
                #if neighborNode is not in nodesToExplore
                    #nodesToExplore.insert(neighborNode)
        
    print("no route")
    return []
        

def constructPath(cameFromDict, currentNode):
    print("constructing Path")
    totalPath = [currentNode]
    while currentNode in cameFromDict:
        currentNode = cameFromDict[currentNode]
        totalPath.append(currentNode)
    return totalPath


    #path = []
    #knownCostOfPath = [math.inf]
    #bestGuessCostOfPath = [h(start)]

def heuristicFunction(startNode, endNode):
    return math.sqrt(math.pow((endNode.x - startNode.x), 2) + math.pow((endNode.y - startNode.y), 2))
    



if __name__ == '__main__':
    main()


"""

function reconstruct_path(cameFrom, current)
    total_path := {current}
    while current in cameFrom.Keys:
        current := cameFrom[current]
        total_path.prepend(current)
    return total_path

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
function A_Star(start, goal, h)

    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore := map with default value of Infinity
    gScore[start] := 0

    // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how short a path from start to finish can be if it goes through n.
    fScore := map with default value of Infinity
    fScore[start] := h(start)

    while openSet is not empty
        // This operation can occur in O(1) time if openSet is a min-heap or a priority queue
        current := the node in openSet having the lowest fScore[] value
        if current = goal
            return reconstruct_path(cameFrom, current)

        openSet.Remove(current)
        for each neighbor of current
            // d(current,neighbor) is the weight of the edge from current to neighbor
            // tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore := gScore[current] + d(current, neighbor)
            if tentative_gScore < gScore[neighbor]
                // This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] := current
                gScore[neighbor] := tentative_gScore
                fScore[neighbor] := tentative_gScore + h(neighbor)
                if neighbor not in openSet
                    openSet.add(neighbor)

    // Open set is empty but goal was never reached
    return failure """