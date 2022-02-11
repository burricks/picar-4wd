#import picar_4wd as fc
import math
import sys
import numpy as np
#from advanced_map import main as adv_map
#from picar_4wd.speed import Speed
import time
from enum import IntEnum
from helps import PriorityQueue
from helps import Node
from helps import NodeMap

"""
    myQueue = PriorityQueue()
    myQueue.insert(12)
    myQueue.insert(1)
    myQueue.insert(14)
    myQueue.insert(7)
    print(myQueue)            
    while not myQueue.isEmpty():
        print(myQueue.delete()) """


def main():
    startNode = Node(0,0)
    goalNode = Node(60,60)
    astar(startNode, goalNode, heuristicFunction)


def astar(startNode, goalNode, heuristicFunction):
    nodesToExplore = PriorityQueue()
    nodesToExplore.insert(startNode)

    nodeMap = NodeMap()
    nodeMap.insert(startNode)
    nodeMap.setGScore(0, 0)
    nodeMap.setFScore(heuristicFunction(startNode, goalNode), 0)
    print(nodeMap)

    while not nodesToExplore.isEmpty():
        currNode = nodesToExplore.delete()
        if currNode == goalNode: #how to determine if I am at the goal
            return True #reconstruct path? Or just be done?
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

    return False
        


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