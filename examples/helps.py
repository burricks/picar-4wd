import math
from enum import IntEnum

#https://www.geeksforgeeks.org/priority-queue-in-python/
class PriorityQueue(object):
    def __init__(self):
        self.queue = []
  
    def __str__(self):
        for i in self.queue:
            print(str(i))
  
    # for checking if the queue is empty
    def isEmpty(self):
        return len(self.queue) == 0
  
    # for inserting an element in the queue
    def insert(self, data):
        self.queue.append(data)
  
    # for popping an element based on Priority
    def delete(self):
        try:
            max = 0
            for i in range(len(self.queue)):
                if self.queue[i].gt(self.queue[max]):
                    max = i
            item = self.queue[max]
            del self.queue[max]
            return item
        except IndexError:
            print()
            exit()


class NodeMap(object):
    def __init__(self):
        self.list = []
     
    def __getitem__(self, item):
        return self.list[item]

    def __str__(self):
        return ' '.join([str(i) for i in self.list])

    def isEmpty(self):
        return len(self.list) == 0

    def nodeInNodeMapIndex(self, node):
        for i in range(0, len(self.list)):
            currNode = self.list[i].node
            if currNode.x == node.x and currNode.y == node.y:
                return i
        return -1

    def insert(self, node):
        existingIndex = self.nodeInNodeMapIndex(node)
        if existingIndex > 0:
            return existingIndex
        nodeObjectToInsert = NodeObject(node, len(self.list))
        self.list.append(nodeObjectToInsert)
        return nodeObjectToInsert.index

    def getNodeObject(self, index):
        try:
            nodeObjectToGet = self.list[index]
            if nodeObjectToGet.deleted:
                return None
            return nodeObjectToGet
        except IndexError:
            return None

    def getFirstUndeletedNode(self):
        for i in len(self.list):
            currNode = self.getNode(i)
            if not currNode.deleted:
                return currNode
        else:
            return None

    def printNodesAndFScores(self, indices):
        allNodeObjects = []
        #return ' '.join([str(i) for i in self.list])
        for index in indices:
            allNodeObjects.append(self.getNodeObject(index))
        print(' '.join([str(i.node) + ": " + str(i.fScore) for i in allNodeObjects]))



    def getBestNodeObject(self, indices):
        best = indices[0]
        if len(indices) < 2:
            return self.getNodeObject(best)
        for i in range(1, len(indices)):
            #print (str(self.getNodeObject(indices[i])))
            currBestNodeObject = self.getNodeObject(best)
            rivalBestNodeObject = self.getNodeObject(indices[i])
            if rivalBestNodeObject.fScore < currBestNodeObject.fScore:
                best = indices[i]
            elif rivalBestNodeObject.fScore == currBestNodeObject.fScore:
                if currBestNodeObject.node.y < rivalBestNodeObject.node.y: #we favor the node that gets us to move forward
                    best = indices[i]
        return self.getNodeObject(best)
        """ firstUndeletedNode = self.getFirstUndeletedNode()
        if firstUndeletedNode is None:
            return None
        else: 
            best = firstUndeletedNode.index
            for i in len(self.list):
                if self.getNode(i).gScore < self.getNode(best).gScore
                best = i
            return self.getNode(best) """


    def deleteNode(self, index):
        nodeObject = self.list.getNodeObject(index)
        nodeObject.deleted = True

    def setGScore(self, gScore, index):
        nodeObjectToSet = self.list[index]
        nodeObjectToSet.gScore = gScore

    def setFScore(self, fScore, index):
        nodeObjectToSet = self.list[index]
        nodeObjectToSet.fScore = fScore

    def setCameFromNode(self, cameFromNode, index):
        nodeObjectToSet = self.list[index]
        nodeObjectToSet.cameFromNode = cameFromNode

class NodeObject(object):
    def __init__(self, node, index):
        self.node = node
        self.index = index
        self.deleted = False
        self.gScore = math.inf
        self.cameFromNode = None
        self.fScore =  math.inf

    def __gt__(self, other):
        return self.fScore > other.fScore

    def __str__(self):
        return(str(self.index) + ": (" + str(self.node.x) + "," + str(self.node.y) + ") Deleted: " + str(self.deleted) + " GScore: " + str(self.gScore) + " FScore: " + str(self.fScore))

class Node(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return("(" + str(self.x) + "," + str(self.y) + ")")

        self.servo.set_angle(angle)
        time.sleep(0.12)
        distance = self.get_distance()
        self.angle_distance = [angle, distance]
        return distance

class Direction(IntEnum):
    LEFT = 2
    FORWARD = 1
    RIGHT = 0
    BACKWARD = 3


