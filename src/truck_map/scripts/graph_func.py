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
from os.path import dirname, abspath
from math import sqrt, radians, degrees, ceil
from enum import Enum
import matplotlib.pyplot as plt


SCALE = 10  # Savefile is in mm, and Graph in cm


# For representing a Point in a coordinate system
class Point:

    # Takes (x, y)-coordinates for a Point
    def __init__(self, x, y):
        self.x = x
        self.y = y


    # String representation of a Point object
    def __str__(self):
        to_str = "%s, %s" % (self.x, self.y)
        return to_str


# For representing a Directed Graph
class Graph:

    # Takes a Dictionary of Node objects, which make up the Graph (optional)
    def __init__(self, nodes=None):
        self.nodes = nodes if nodes else dict()


    # String representation of a Graph object
    def __str__(self):
        to_str = ""
        nodes = self.nodes.values()

        for node in nodes[:-1]:
            to_str += "%s\n" % node
        if len(nodes) > 0:
            to_str += "%s" % nodes[-1]
        return to_str


    # Takes another Graph object
    # Adds all Nodes from that Graph to this Graph (without creating duplicates)
    def addGraph(self, graph):
        for node in graph.nodes:
            self.addNode(node)


    # Takes a Node object
    # Adds that Node to this Graph (without creating duplicates)
    def addNode(self, node):
        current_node = self.getNode(node.x, node.y)
        if current_node:
            for e in node.out_edges:
                self.addNode(e)
                current_node.addOutEdge(e)
        else:
            self.nodes[(node.x, node.y)] = node


    # Takes (x, y)-coordinates for a Node
    #
    # If there is a Node object with given coordinates in this Graph:
    #     Returns that Node object
    # Otherwise:
    #     Reurns None 
    def getNode(self, x, y):
        return self.nodes.get((x, y))


    # For resetting the parameters used by shortestPath()
    def resetGraph(self):
        for node in self.nodes.values():
            node.count = 0


# For representing a Node in a Graph
class Node:

    # Takes (x, y)-coordinates for this Node
    # and an array of Node objects, to which there is an out-edge from this Node (optional)
    def __init__(self, x, y, out_edges=None):
        self.x = x
        self.y = y
        self.out_edges = out_edges if out_edges else []

        # For use in shortestPath()
        self.count = 0


    # String representation of a Node object
    def __str__(self):
        to_str = "(%s, %s) [" % (self.x, self.y)
        for node in self.out_edges[:-1]:
            to_str += "(%s, %s) " % (node.x, node.y)
        if len(self.out_edges) > 0:
            to_str += "(%s, %s)" % (self.out_edges[-1].x, self.out_edges[-1].y)
        to_str += "]"
        return to_str


    # Definition of equality between two Node objects
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


    # Takes another Node object
    # Adds an outgoing edge from this Node to that Node (without creating duplicates)
    def addOutEdge(self, node):
        out_edge = self.getOutEdge(node.x, node.y)
        if not out_edge:
            self.out_edges.append(node)


    # Takes (x,y)-coordinates for a Node
    #
    # If there is a Node object with given coordinates in the out-edges from this Node:
    #     Returns that Node object
    # Otherwise:
    #     Reurns None 
    def getOutEdge(self, x, y):
        for node in self.out_edges:
            if node.x == x and node.y == y:
                # Node object found
                return node
        # Node object Not found
        return None


    # Returns the length of an edge between this Node and the given Node object
    def getEdgeLength(self, to_node):
        return sqrt((to_node.x - self.x)**2 + (to_node.y - self.y)**2)


# Takes a path (relative to current directory), to a text file containing a Graph representation
# (path='/graph.txt' for file 'graph.txt', located in current directory)
#
# The textfile should be in the format specified in 'example_graph.txt'
#
# The textfile is assumed to store Graph in mm
# The Graph returned by this function is in cm
#
# If parsing was completed without syntax errors:
#     Returns a Graph object
# Otherwise:
#     Returns None
def readFileToGraph(path):
    graph = Graph()
    current_node = None
    dirpath = dirname(abspath(__file__))
    line_counter = 0

    # Variables for validation of syntax
    begin = False
    valid = False


    # Opening the file, parsing the lines one by one
    with open(dirpath + path, 'r') as file:
        for line in file:
            # Line counter, for debugging of input file
            line_counter += 1
            # Removing whitespaces and tabs
            line = line.replace(' ', '')
            line = line.replace('\t', '')

            # Ignoring comments and empty lines
            if not line.startswith('#') and not line.startswith('\n'):

                # Start of node declaration
                if line == "NODE\n":
                    if not begin:
                        begin = True
                    else:
                        print "Syntax error on line %s in '%s'" % (line_counter, path)
                        return None

                # End of node declaration
                elif line == "ENDNODE\n":
                    if begin and current_node:
                        begin = False
                        valid = False
                        current_node = None
                    else:
                        print "Syntax error on line %s in '%s'" % (line_counter, path)
                        return None

                # Coordinates for current Node and its out-edges (connected Nodes)
                elif line[0].isdigit():

                    # Validating syntax
                    if begin:
                        # Removing newline characters and splitting line on list separator ';'
                        array = line.replace('\n', '').split(';')

                        # Out-edges
                        if current_node:
                            for elem in array:
                                elem = elem.split(',')
                                valid = True if len(elem) == 2 and elem[0].isdigit() and elem[1].isdigit() else False
                                # Breaking on Syntax error
                                if not valid:
                                    break

                                (x, y) = (float(elem[0]) / SCALE, float(elem[1]) / SCALE)
                                # If the connected Node is not already in the node-list, create a new Node object for it
                                outedge = graph.getNode(x, y)
                                if not outedge:
                                    outedge = Node(x, y)
                                    graph.addNode(outedge)
                                current_node.addOutEdge(outedge)

                        # Current Node
                        elif len(array) == 1:
                            elem = array[0].split(',')
                            valid = True if len(elem) == 2 and elem[1].isdigit() else False
                            # Breaking on Syntax error
                            if not valid:
                                break

                            (x, y) = (float(elem[0]) / SCALE, float(elem[1]) / SCALE)
                            # If the current Node is not already in the node-list, create a new Node object for it
                            current_node = graph.getNode(x, y)
                            if not current_node:
                                current_node = Node(x, y)
                                graph.addNode(current_node)

                        # Syntax error
                        else:
                            valid = False

                    # Syntax error
                    if not valid:
                        print "Syntax error on line %s in '%s'" % (line_counter, path)
                        return None

                # Syntax error
                else:
                    print "Syntax error on line %s in '%s'" % (line_counter, path)
                    return None

        # Syntax error
        if begin:
            print "Syntax error on line %s in '%s'" % (line_counter, path)
            return None

        return graph


# Takes a Graph and a filename
# Stores the Graph data in a file with given filename,
# in a format that can be re-read by calling 'readFileToGraph(path_to_savefile)'
#
# The given Graph is assumed to be in cm
# The textfile created by this function stores the Graph in mm, with one decimal's accuracy and rounded up
def saveGraphToFile(graph, filename):
    with open(filename, 'w') as file:

        for node in graph.nodes.values():
            file.write("NODE\n")
            file.write("    %s,%s\n" % (int(ceil(node.x * SCALE)), int(ceil(node.y * SCALE))))

            for out_edge in node.out_edges[:1]:
                file.write("    %s,%s" % (int(ceil(out_edge.x * SCALE)), int(ceil(out_edge.y * SCALE))))
            for out_edge in node.out_edges[1:]:
                file.write(" ; %s,%s" % (int(ceil(out_edge.x * SCALE)), int(ceil(out_edge.y * SCALE))))

            if node.out_edges:
                file.write('\n')
            file.write("ENDNODE\n\n")


# Takes an array of Point objects
#
# The given points are assumed to be in mm
# The Graph returned by this function is in cm
#
# Returns a Directed Graph, with an edge from each Point to the next one in the array
def pointsToGraphMM(points):
    # Converting to cm
    pts = map(lambda p: Point(float(p.x) / SCALE, float(p.y) / SCALE), points)
    graph = Graph()

    # Creating Node objects for all points
    for point in pts:
        graph.addNode(Node(point.x, point.y))
    
    # Creating an edge from each Node to the next one in the array
    for i in range(len(pts)-1):
        current_node = graph.getNode(pts[i].x, pts[i].y)
        next_node = graph.getNode(pts[i+1].x, pts[i+1].y)
        current_node.addOutEdge(next_node)

    return graph


# Takes an array of Point objects
#
# Given points are assumed to be in cm
# The Graph returned by this function is in cm
#
# Returns a Directed Graph, with an edge from each Point to the next one in the array
def pointsToGraphCM(points):
    graph = Graph()

    # Creating Node objects for all points
    for point in points:
        graph.addNode(Node(float(point.x), float(point.y)))
    
    # Creating an edge from each Node to the next one in the array
    for i in range(len(points)-1):
        current_node = graph.getNode(points[i].x, points[i].y)
        next_node = graph.getNode(points[i+1].x, points[i+1].y)
        current_node.addOutEdge(next_node)

    return graph


# For plotting a Graph
# (color='b' for blue, color='k' for black, etc)
def plotGraph(graph, color, arrowheads=False):
    ax = plt.axes()

    # Plotting all graph edges
    for node in graph.nodes.values():
        for out_edge in node.out_edges:
            dx = out_edge.x - node.x
            dy = out_edge.y - node.y

            if arrowheads:
                ax.arrow(node.x, node.y, dx, dy, head_width=8, head_length=10, fc=color, ec=color)
            else:
                ax.arrow(node.x, node.y, dx, dy, fc=color, ec=color)
