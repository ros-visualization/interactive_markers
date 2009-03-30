#!/usr/bin/env python

##############################################################################
# Copyright (C) 2008, Morgan Quigley
#
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright notice, 
#     this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright 
#     notice, this list of conditions and the following disclaimer in the 
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Stanford University nor the names of its 
#     contributors may be used to endorse or promote products derived from 
#     this software without specific prior written permission.
#   
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.
##############################################################################



import roslib; 
roslib.load_manifest('rospy')
roslib.update_path('multitouch_nav')

import rospy

import sys

import pygame
from pygame.locals import *
pygame.init()

from map import Map
from robot import Robot
#from robotCoordinator import RobotCoordinator
from table import Table
from configSpace import GridConfigSpace
from pathPlanner import AStarPathPlanner
from event import *

NAME = 'multiTouchRobotCommander'
FRAME_RATE = 60
 


def parseWorldFile(fileName):
    """
    Parse a .world file for 4 parameters: the map file, its size (in 
    world coordinates), the number of robots, and the robots' sizes.
    """

    try:
        worldFile = file(fileName, 'r')

        def parseSize3(s):
            """Parse a size3 list"""
            s = s[s.index('size3'):]
            s = s[s.index('[')+1:]
            x = float(s[0:s.index(' ')])
            s = s[s.index(' ')+1:]
            y = float(s[0:s.index(' ')])
            return (x, y, s)

        s = worldFile.read() 

        # get robot size
        s = s[s.index('position'):]
        (x,y,s) = parseSize3(s);
        robotSize = [x,y]

        # get relative path to map file
        s = s[s.index('floorplan\n'):]
        s = s[s.index('bitmap'):]
        s = s[s.index('"')+1:]
        relPath = s[0:s.index('"')]
        
        # get absolute path
        slashIndex = -1
        for i in range(len(fileName)):
            if fileName[i] == '/':
                slashIndex = i
        mapFile = fileName[:slashIndex+1] + relPath

        # get map size
        mapSize = parseSize3(s)[:2]

        # get number of robots
        numRobots = 0
        while True:
            try:
                s = s[s.index('pos'):]
                s = s[s.index('name'):]
                s = s[s.index('color'):]              
                numRobots += 1
            except:
                break

        return (mapFile, mapSize, numRobots, robotSize)

    except:
        raise RuntimeError("error parsing world file")


if __name__ == '__main__':

    # check for world file arg
    if len(sys.argv) < 2:
        print "usage: python multitouch_nav.py [-f] <world>"
        sys.exit()

    if sys.argv[1] == '-f':
        fullScreen = True
    else:
        fullScreen = False

    # extract the file name and size of the map image
    try:
        mapFile, mapSize, numRobots, robotSize = parseWorldFile( sys.argv[len(sys.argv)-1] )
    except RuntimeError, e:
        print e
        sys.exit()

    # create map window
    map = Map( mapFile, mapSize, robotSize, fullScreen )

    # initialize node
    rospy.init_node( NAME, anonymous=True )

    # create clock to control frame rate
    clock = pygame.time.Clock()

    # create robots
    robots = []
#    robotCoordinator = RobotCoordinator()
    for i in range(numRobots):
        r = Robot( i )
        robots.append( r )
#        robotCoordinator.registerRobot( r )

    # pygame/touchKit event handler
    eventQueue = EventQueue(map.getScreenSize())

    configSpace = GridConfigSpace(mapSize, robotSize)
    pathPlanner = AStarPathPlanner(configSpace)
    
    while not rospy.is_shutdown():

        # a map from finger IDs to paths we need to draw
        paths = {}

        for event in eventQueue.popEvents():

            if isinstance(event, QuitEvent):
                sys.exit()

            elif isinstance(event, ZoomInEvent):
                map.zoomIn(event.extent)
                
            elif isinstance(event, ZoomOutEvent):
                map.zoomOut(event.extent)

            elif isinstance(event, PanEvent):
                map.pan(event.dx, event.dy)

            elif isinstance(event, SelectEvent):
                for robot in robots:
                    mapPos = map.mapToScreenCoords(robot.getPos())
                    if event.containsPt(mapPos):
                        robot.setSelected(True)
            
            elif isinstance(event, FingerDownEvent) or isinstance(event, FingerDragEvent):
                if isinstance(event, FingerDownEvent):
                    selectedRobots = map.doHitTest(event.path[-1], robots)
                    for robot in selectedRobots:
                        robot.toggleSelected()
                if isinstance(event, FingerDragEvent) or not selectedRobots:
                    paths[event.id] = event.path

            elif isinstance(event, FingerUpEvent):
                stagePath = map.screenPathToMapPath(event.path)
                for robot in robots:
                    if robot.isSelected():
                        robot.setPath(pathPlanner.getRobotPath(stagePath))
                        robot.go()

        # draw screen
        clock.tick(FRAME_RATE)
        map.repaint( robots, [paths[id] for id in paths.keys()] )

        # for debugging...
        for robot in robots:
            if not configSpace.isFree(robot.getPos()):
                print "illegal pos: ", robot.getPos()
