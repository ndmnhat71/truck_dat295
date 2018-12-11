#!/usr/bin/env python
"""
Copyright (c) 2017, Lars Niklasson
Copyright (c) 2017, Filip Slottner Seholm
Copyright (c) 2017, Fanny Sandblom
Copyright (c) 2017, Kevin Hoogendijk
Copyright (c) 2017, Nils Andren
Copyright (c) 2017, Alicia Gil Martin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Chalmers University of Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
from graph_func import *
from heapq import heappush, heappop


# Takes a Graph and two Point objects with (x, y)-coordinates for start and end point
#
# If there is a path from the start Point to the end Point:
#     Returns the shortest path between them as an array of tuples of (x, y)-coordinates
# Otherwise:
#     Returns None
def shortestPath(graph, start, end):

    # Used to specify search range for finding closest point
    search_range = 20
    nodes = graph.nodes.values()

    # Finding start resp. end Node:

    start_node = graph.getNode(start.x, start.y)

    # If the coordinates for the start point does Not exactly match a Node in the graph
    if not start_node:

        # Selecting all Nodes which are in range from the start point
        nodes = getAllInRangeX(getAllInRangeY(nodes, start, search_range), start, search_range)
        # Returning None if No Node is in range
        if not nodes:
            return None
        # Finding the two Nodes which are closest to the start point, x-wise resp. y-wise
        closest_x = getClosestX(nodes, start)
        closest_y = getClosestY(nodes, start)
        # Selecting the Node which is closest to the start point
        dx = Node(start.x, start.y).getEdgeLength(closest_x)
        dy = Node(start.x, start.y).getEdgeLength(closest_y)
        start_node = closest_x if dx <= dy else closest_y

    end_node = graph.getNode(end.x, end.y)

    # If the coordinates for the end point does Not exactly match a Node in the graph
    if not end_node:

        # Selecting all Nodes which are in range from the end point
        nodes = getAllInRangeX(getAllInRangeY(nodes, end, search_range), end, search_range)
        # Returning None if No Node is in range
        if not nodes:
            return None
        # Finding the two Nodes which are closest to the end point, x-wise resp. y-wise
        closest_x = getClosestX(nodes, end)
        closest_y = getClosestY(nodes, end)
        # Selecting the Node which is closest to the end point
        dx = Node(end.x, end.y).getEdgeLength(closest_x)
        dy = Node(end.x, end.y).getEdgeLength(closest_y)
        end_node = closest_x if dx <= dy else closest_y

    path_list = kShortestPaths(graph, start_node, end_node, 1)

    if path_list:
        return path_list[0]
    else:
        return path_list


# Takes a Graph, two Point objects with (x, y)-coordinates for start and end point,
# and the desired number of alternative paths
#
# The given start and end Points have to exactly match Nodes in the given Graph
#
# If given paramaters are invalid:
#     Returns None
# If there is at least one alternative path between the given start and end points:
#     Returns an array of (at most k) alternative paths between them, in order of length (increasing),
#     where each path is an array of tuples of (x, y)-coordinates
# Otherwise:
#     Returns []
def altPaths(graph, start, end, k):
    start_node = graph.getNode(start.x, start.y)
    end_node = graph.getNode(end.x, end.y)

    # Returning None if the given start and end Points do Not exactly match Nodes in the given Graph
    if not start_node or not end_node:
        return None

    paths = kShortestPaths(graph, start_node, end_node, k+1)

    # If there are any alternative paths, returns all (at most k) alternative paths found
    if paths:
        return paths[1:]
    else:
        return []


# Takes a Graph, two Nodes for start and end point, the desired number of shortest paths,
# and a Boolean that decides if loops should be allowed or not (optional)
#
# By default, loops (i.e. the same Node being visited more than once) are Not allowed
#
# If the start Node is the same as the end Node:
#     Returns []
# If there is at least one path from the start Node to the end Node:
#     Returns an array of (at most k) shortest paths between them, in order of length (increasing),
#     where each path is an array of tuples of (x, y)-coordinates
# Otherwise:
#     Returns None
def kShortestPaths(graph, start_node, end_node, k, allow_loops=False):
    if start_node == end_node:
        return []

    path_heap = []
    paths = []

    graph.resetGraph()
    heappush(path_heap, (0, [(start_node.x, start_node.y)]))

    # Repeating until the end Node has been visited k times
    # (or until we know that there is no more paths from the start Node to the end Node)
    while end_node.count < k and path_heap:
        cost, path = heappop(path_heap)
        current_node = graph.getNode(path[-1][0], path[-1][1])
        current_node.count += 1

        # If the end Node has been reached, adding this path to the array of shortest paths
        if current_node == end_node:
            paths.append(path)

        # Otherwise if the current Node is not already in all k shortest paths,
        # adding all possible paths forward from this Node to the path heap
        elif current_node.count <= k or allow_loops:
            for out_edge in current_node.out_edges:
                if (out_edge.x, out_edge.y) not in path or allow_loops:
                    new_path = []
                    for coord in path:
                        new_path.append(coord)
                    new_path.append((out_edge.x, out_edge.y))
                    new_cost = cost + current_node.getEdgeLength(out_edge)
                    heappush(path_heap, (new_cost, new_path))

    if paths:
        return paths
    # Returns None if No paths were found
    else:
        return None


# Takes a Graph and a VehicleState object
#
# If there is at least one Node with an out-edge in the right Direction (with respect to theta):
#     Returns the closest Node which has an out-edge in the right Direction
# Otherwise:
#     Returns None
def getClosestToVehicle(graph, vehicle_state):

    # Used to specify search range for finding closest point
    search_range = 100

    # Normalizing theta
    theta = degrees(vehicle_state.theta1) % 360
    pos = Point(vehicle_state.x, vehicle_state.y)
    nodes = graph.nodes.values()

    # Vehicle angle: bottom-to-top
    if theta > 225 and theta <= 315:
        # Selecting all Nodes which are in range from the vehicle
        nodes = getAllInRangeX(getAllInRangeY(nodes, pos, search_range, 0), pos, search_range)
        # Selecting all Nodes which have an out-edge upwards from vehicle position
        nodes = getAllInRightDir(nodes, pos, Direction.up)

    # Vehicle angle: right-to-left
    elif theta > 135 and theta <= 225:
        # Selecting all Nodes which are in range from the vehicle
        nodes = getAllInRangeX(getAllInRangeY(nodes, pos, search_range), pos, search_range, 0)
        # Selecting all Nodes which have an out-edge to the left from vehicle position
        nodes = getAllInRightDir(nodes, pos, Direction.left)

    # Vehicle angle: top-to-bottom
    elif theta > 45 and theta <= 135:
        # Selecting all Nodes which are in range from the vehicle
        nodes = getAllInRangeX(getAllInRangeY(nodes, pos, 0, search_range), pos, search_range)
        # Selecting all Nodes which have an out-edge downwards from vehicle position
        nodes = getAllInRightDir(nodes, pos, Direction.down)

    # Vehicle angle: left-to-right
    elif theta > 315 or theta <=45:
        # Selecting all Nodes which are in range from the vehicle
        nodes = getAllInRangeX(getAllInRangeY(nodes, pos, search_range), pos, 0, search_range)
        # Selecting all Nodes which have an out-edge to the right from vehicle position
        nodes = getAllInRightDir(nodes, pos, Direction.right)

    # Returning None if No Node is in range
    if not nodes:
        return None
        
    # Finding the two Nodes which are closest to the vehicle, x-wise resp. y-wise
    closest_x = getClosestX(nodes, pos)
    closest_y = getClosestY(nodes, pos)
    # Selecting the Node which is closest to the vehicle:
    dx = Node(pos.x, pos.y).getEdgeLength(closest_x)
    dy = Node(pos.x, pos.y).getEdgeLength(closest_y)
    start_node = closest_x if dx <= dy else closest_y

    return start_node


# Takes an array of Node objects, a Point object, and search range limits
# (to the left resp. to the right of given Point)
#
# Returns an array with all Nodes that are in range (set by given limits) x-wise from given Point
def getAllInRangeX(nodes, point, range_left, range_right=None):
    range_right = range_right if range_right != None else range_left
    node_list = []
    for node in nodes:

        # Selecting all Nodes that are in range x-wise
        if node.x >= point.x-range_left and node.x <= point.x+range_right:
            node_list.append(node)

    return node_list


# Takes an array of Node objects, a Point object, and search range limits
# (above resp. below given Point)
#
# Returns an array with all Nodes that are in range (set by given 'limit') y-wise from given Point
def getAllInRangeY(nodes, point, range_above, range_below=None):
    range_below = range_below if range_below != None else range_above
    node_list = []
    for node in nodes:

        # Selecting all Nodes that are in range y-wise
        if node.y >= point.y-range_above and node.y <= point.y+range_below:
            node_list.append(node)

    return node_list


# Takes an array of Node objects and a Point object
#
# Returns the Node that is (x-wise) closest to given Point
def getClosestX(nodes, point):
    closest_node = None
    closest_dist = float('inf')

    for node in nodes:
        dist = abs(point.x - node.x)

        # If no Node can be closer
        if dist == 0:
            return node
        elif dist < closest_dist:
            closest_dist = dist
            closest_node = node

    return closest_node


# Takes an array of Node objects and a Point object
#
# Returns the Node that is (y-wise) closest to given Point
def getClosestY(nodes, point):
    closest_node = None
    closest_dist = float('inf')
    
    for node in nodes:
        dist = abs(point.y - node.y)

        # If no Node can be closer
        if dist == 0:
            return node
        elif dist < closest_dist:
            closest_dist = dist
            closest_node = node

    return closest_node


Direction = Enum('Direction', 'up down left right')


# Takes an array of Node objects, a Point object and a Direction
#
# Returns an array with all Nodes that have an out-edge in the given Direction
def getAllInRightDir(nodes, point, direction):
    node_list = []
    for node in nodes:

        # Selecting all Nodes that have an out-edge in the right direction
        if hasOutEdgeInRightDir(node, direction):
            node_list.append(node)

    return node_list


# Takes a Node object and a Direction
#
# If given Node has an out-edge in given Direction:
#     Returns True
# Otherwise:
#     Returns False
def hasOutEdgeInRightDir(node, direction):

    # Used to specify minimum difference between given Node and an out-edge-node,
    # (x-wise resp. y-wise) for the edge to be considered as going in a specific Direction
    min_offset = 5

    # Going through all out-edges,
    # returning True as soon as we find one that goes in the given Direction
    for out_edge in node.out_edges:

        if direction == Direction.up:
            # If given Node has an out-edge upwards
            if (node.y - out_edge.y) > min_offset:
                return True

        elif direction == Direction.down:
            # If given Node has an out-edge downwards
            if (node.y - out_edge.y) < -min_offset:
                return True

        elif direction == Direction.left:
            # If given Node has an out-edge to the left
            if (node.x - out_edge.x) > min_offset: 
                return True

        elif direction == Direction.right:
            # If given Node has an out-edge to the right
            if (node.x - out_edge.x) < -min_offset:
                return True 

    # No out-edge in the given Direction
    return False
