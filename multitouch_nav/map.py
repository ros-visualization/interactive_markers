import pygame, math, os
from table import Table
from pygame.locals import *
import utils



class Map:


    # drawing contants
    PATH_THICKNESS = 2
    FINGER_PATH_COLOR = utils.PURPLE
    FINGER_COLOR = utils.WHITE
    FINGER_RADIUS = 4
    GOAL_RADIUS = 5
    GOAL_COLOR = utils.YELLOW
    ROBOT_PATH_COLOR = utils.RED
    ROBOT_COLOR = utils.GREEN
    SELECTED_ROBOT_COLOR = utils.RED
    
    # controls how fast we zoom in/out
    ZOOM_RATE = 1.3



    def __init__(self, mapFile, mapSize, robotSize, fullScreen=False):

#        # first try loading the larger png file if one exists
#        png = mapFile[:-3]+"png"
#        if os.path.exists(png):
#            pngImg = pygame.image.load(png)
#            self._origMapImage = pygame.transform.rotate(pngImg, -90)
#        else:
        self._origMapImage = pygame.image.load(mapFile)
        if not self._origMapImage:
            print "error loading map file"
            sys.exit()

        # size of the map in stage coordinates (not screen coordinates)
        self._mapSize = mapSize[:]

        # size to draw the robots (in stage coords)
        self._robotSize = robotSize

        self._screenSize = (self._origMapImage.get_rect().width, self._origMapImage.get_rect().height)
        if fullScreen:
            self._screen = pygame.display.set_mode(self._screenSize, FULLSCREEN)
        else:
            self._screen = pygame.display.set_mode(self._screenSize)

        self._zoomMapImage = pygame.transform.scale(self._origMapImage, self._screenSize)

        self._scale = 1.0
        self._zoomOffset = (0,0)   # the screen coordinates where we start blitting the zoomed image, ignoring pan
        self._panOffset = (0,0)



    def getScreenSize(self):
        return self._screenSize



    def _getBlitCoords(self):
        return [ self._panOffset[i] + self._zoomOffset[i] for i in range(2) ]



    def _getZoomedImageSize(self):
        return [ self._scale * self._screenSize[i] for i in range(2) ]



    def repaint(self, robots, paths):
        self._screen.fill(utils.BLACK)
        self._drawMap()
        for robot in robots:
            self._drawRobotIcon(robot)
            self._drawRobotPath(robot.getPath())
        for path in paths:
            self._drawFingerPath(path)
        pygame.display.flip()



    def _drawMap(self):
        self._screen.blit(self._zoomMapImage, self._getBlitCoords())



    def _drawRobotPath(self, path):
        path = self.mapPathToScreenPath(path)
        self._drawPath(path, Map.ROBOT_PATH_COLOR, Map.GOAL_COLOR, Map.GOAL_RADIUS)



    def _drawFingerPath(self, path):
        self._drawPath(path, Map.FINGER_PATH_COLOR, Map.FINGER_COLOR, Map.FINGER_RADIUS)     



    def _drawPath(self, path, pathColor, endColor, endRadius):
        """Draws a path, given in screen coordinates."""
        if not path: return
        path = [(int(x), int(y)) for x,y in path]
        if len(path) > 1:
            pygame.draw.lines(self._screen, pathColor, False, path, Map.PATH_THICKNESS)
        pygame.draw.circle(self._screen, endColor, path[len(path)-1], endRadius)



    def screenToMapCoords(self, mousePos):
        mousePos = [ mousePos[i] - self._getBlitCoords()[i] for i in range(2) ]
        return utils.scaleCoords(mousePos, self._getZoomedImageSize(), self._mapSize)



    def mapToScreenCoords(self, mapPos):
        scaled = utils.scaleCoords(mapPos, self._mapSize, self._getZoomedImageSize() )
        return [ scaled[i] + self._getBlitCoords()[i] for i in range(2) ]



    def screenPathToMapPath(self, path):
        return [self.screenToMapCoords(pt) for pt in path]


    
    def mapPathToScreenPath(self, path):
        return [self.mapToScreenCoords(pt) for pt in path]



    def _drawRobotIcon(self, robot):

        # get size and position of icon
        x, y = self.mapToScreenCoords( robot.getPos() )
        iconDims = [ self._screenSize[i] / self._mapSize[i] *\
                         self._scale * self._robotSize[i] for i in range(2) ]

        # draw green robot rect
        surface = pygame.Surface( iconDims, flags=SRCALPHA )
        rect = ( 0, 0, iconDims[0], iconDims[1] )
        color = robot.isSelected() and Map.SELECTED_ROBOT_COLOR or Map.ROBOT_COLOR
        pygame.draw.rect( surface, color, rect )

        # put black dot at front of robot
        dotCenter = ( iconDims[0]*.95, iconDims[1]*.5 )
        pygame.draw.circle( surface, utils.BLACK, dotCenter, int(iconDims[0]*.1) )

        # rotate according to theta and blit to screen
        deg = math.degrees( robot.getTheta() )
        surface = pygame.transform.rotate( surface, deg )
        x -= surface.get_rect().width/2.
        y -= surface.get_rect().height/2.
        self._screen.blit( surface, (x,y) )



    def zoomIn(self, extent):
        self._scale *= extent * Map.ZOOM_RATE
        self._recomputeZoom()



    def zoomOut(self, extent):
        self._scale /= extent * Map.ZOOM_RATE
        self._recomputeZoom()



    def pan(self, dx, dy):
        self._panOffset = [ self._panOffset[i] + [dx,dy][i] for i in range(2) ]



    def _recomputeZoom(self):
        newDims = [ int(self._screenSize[i] * self._scale) for i in range(2) ]
        self._zoomMapImage = pygame.transform.scale(self._origMapImage, newDims)
        self._zoomOffset = [ self._screenSize[i] * (1.-self._scale)/2. for i in range(2) ]


    def doHitTest(self, pt, robots):
        """
        Return a list of robots whose icons contain the given point.  Ignore rotation for now.
        Note that pt is in screen coords.
        """
        hits = []
        for robot in robots:
            pos = self.mapToScreenCoords(robot.getPos())
            hitRect = utils.scaleCoords(self._robotSize, self._mapSize, self._screenSize, False)
            minX = pos[0] - hitRect[0]/2.
            maxX = pos[0] + hitRect[0]/2.
            minY = pos[1] - hitRect[1]/2.
            maxY = pos[1] + hitRect[1]/2.
            if pt[0] >= minX and pt[0] <= maxX and \
                    pt[1] >= minY and pt[1] <= maxY:
                hits.append(robot)
        return hits
