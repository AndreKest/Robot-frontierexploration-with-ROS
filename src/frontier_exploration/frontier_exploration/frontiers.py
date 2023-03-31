#!/usr/bin/env python3
# Calculate the frontiers
# Based on the library of Sean Reg and the original paper "Frontier Based Exploration for Autonomous Robot"
# GitHub: https://github.com/SeanReg/nav2_wavefront_frontier_exploration
# Paper: https://arxiv.org/ftp/arxiv/papers/1806/1806.03581.pdf
from enum import Enum

import numpy as np

OCC_THRESHOLD = 30
MIN_FRONTIER_SIZE = 20

class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def to_numpy(self):
        data = np.asarray(self.map.data, dtype=np.int8).reshape(self.getSizeY(), self.getSizeX())
        tmp = np.array(data, dtype=np.uint8)
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                if data[i,j] == 0:
                    tmp[i,j] = 255
                elif data[i,j] == 100:
                    tmp[i,j] = 0
                else:
                    tmp[i,j] = 100

        return tmp

    def mapToWorld(self, mx, my):
        """ Transform map coodiantes to world coordinates """
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        """ Transform world coodiantes to map coordinates """
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx


class FrontierCache():
    cache = {}
    def getPoint(self, x, y):
        idx = self.__cantorHash(x, y)

        if idx in self.cache:
            """ If points are already in the cache -> return it """
            return self.cache[idx]

        # If point are not in the cache, add the point and return it
        self.cache[idx] = FrontierPoint(x,y)
        return self.cache[idx]

    def __cantorHash(self, x, y):
        """ Hash index of the point (x,y) """
        return (((x + y) * (x + y + 1)) / 2) + y
    
    def clear(self):
        """ Delete the Frontier Cache """
        self.cache = {}


class FrontierPoint():
    def __init__(self, x, y):
        """ Frontier Point with x,y coordinates in map space """
        # using binary: MapOpen: 0001 Dec(1) | MapClosed: 0010 Dec(2) | FrontierOpen: 0100 Dec(4) | FrontierClosed: 1000 Dec(8)
        # Examlpe:
        # if MapOpen Flag set and FrontierOpen Flag set then: 0101 Dec(5)
        self.classification = 0 # 0: Init, 1: MapOpen, 2: MapClosed, 4: FrontierOpen, 8: FrontierClosed
        self.mapX = x
        self.mapY = y

def centroid(arr):
    """ Calculate centroid of frontier """
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x/length, sum_y/length


def findFree(mx, my, costmap):
    """ Find surrounding free points of the current robot pose with Breadth-First Search (BFS) in the map """
    fCache = FrontierCache()

    bfs = [fCache.getPoint(mx, my)]

    # expand from current roboter position to find free points
    while len(bfs) > 0:
        loc = bfs.pop(0)

        # Check if costvalue of loc.X,loc.Y == 0 (free space) True. return point
        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)


def getFrontier(pose, costmap):
    """ Based on the current position of the robot (pose) and the map, calculate all possible frontier points """
    fCache = FrontierCache()

    fCache.clear()

    # mx, my current pose of robot in map space
    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)

    # find surrounding free points in costmap
    freePoint = findFree(mx, my, costmap)   # get free point
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = []

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        # Check if p is already marked as MapClosed
        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        if isFrontierPoint(p, costmap, fCache):
            # Mark p additionally as FrontierOpen
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = []

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)
                # Check if q is already marked as MapClosed or FrontierClosed
                if q.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:
                    continue

                if isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        # Check if w is not marked as FrontierOpen, FrontierClosed or MapClosed 
                        if w.classification & (PointClassification.FrontierOpen.value | PointClassification.FrontierClosed.value | PointClassification.MapClosed.value) == 0:
                            # Mark w additonally as FrontierOpen
                            w.classification = w.classification | PointClassification.FrontierOpen.value
                            frontierQueue.append(w)

                # Mark q additionally as FrontierClosed
                q.classification = q.classification | PointClassification.FrontierClosed.value

            
            newFrontierCords = []
            for x in newFrontier:
                # Mark x additionally as MapClosed
                x.classification = x.classification | PointClassification.MapClosed.value
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))

            if len(newFrontier) > MIN_FRONTIER_SIZE:
                frontiers.append(centroid(newFrontierCords))

        # if point p was not a FrontierPoint -> check the surrounding neighbors
        for v in getNeighbors(p, costmap, fCache):
            # if v is marked as MapOpen or MapClosed
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                # Check costmap value of the surrounding neighbors of point v and
                # if any neighbors of v hast cost value FreeSpace (0) then set point v classification value to MapOpen (1) and append to mapPointList
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):
                    # Mark v additionally as MapOpen
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        # Mark p additionally as MapClosed
        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers


def getNeighbors(point, costmap, fCache):
    """ Get surrounding neighbors of the point in the map """
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            # Check if neighbor is inside map, true -> add point to neighbor list
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))

    return neighbors


def isFrontierPoint(point, costmap, fCache):
    """ Check if Point in fCache is valid frontier point """
    # Check cost value of point, 
    # if cost value from point != -1 -> no frontier point return false;
    # if == -1 then  check neighbors because actual point is not known -> has value -1 
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)

        if cost > OCC_THRESHOLD:
            return False

        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree


class PointClassification(Enum):
    """
    Enumeration for the classification of the Frontier Point
    MapOpen: point that have been enqueued by the outer BFS.
    MapClose: point that have been dequeued by the outer BFS.
    FrontierOpen: point that have been enqueued by the inner BFS.
    FrontierClose: point that have been dequeued by the inner BFS.
    """
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8
