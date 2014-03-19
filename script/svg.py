#!/usr/bin/env python

class SvgDrawing (object):
    prefix = '<!DOCTYPE html>\n<html>\n<body>\n<svg height="%i" width="%i">\n'
    suffix = '</svg>\n</body>\n</html>\n'
    def __init__ (self, filename, width = 200, height = 200,
                  xOffset = 100, yOffset = 100, scale = 40):
        self.width = width
        self.height = height
        self.xOffset = xOffset
        self.yOffset = yOffset
        self.scale = scale
        self.filename = filename
        self.file = file (filename, 'w')
        self.file.write (self.prefix%(width, height))
        self.drawBox ()

    def convert (self, xy):
        return [xy [0] * self.scale + self.xOffset,
                xy [1] * self.scale + self.yOffset]

    def __del__ (self):
        self.file.write (self.suffix)
        self.file.close ()

    def drawBox (self):
        self.file.write ('  <rect x="0" y="0" width="%i" height="%i"'%
                         (self.width,self.height) +
                         ' style="fill:black;stroke:green;stroke-width:1'+
                         ';fill-opacity:0.1;stroke-opacity:1.0" />\n')

    def drawCircle (self, q, color):
        x, y = self.convert (q)
        self.file.write ('  <circle cx="%i" cy="%i" r="3" '%(x+.5,y+.5)+
                         'stroke="black" stroke-width="1" fill="%s" ;'%color +
                         'fill-opacity=0.2 />\n')

    def drawRectangle (self, xCenter, yCenter, xLength, yLength):
        x, y = self.convert ([xCenter - .5*xLength, yCenter - .5*yLength])
        w = xLength * self.scale
        h = yLength * self.scale
        self.file.write ('  <rect x="%i" y="%i" width="%i" height="%i" style="fill:black;stroke:pink;stroke-width:0;fill-opacity:0.5;stroke-opacity:0.9"/>\n'%
                         (x, y, w, h))

    def drawLine (self, x1, y1, x2, y2):
        X1, Y1 = self.convert ([x1,y1])
        X2, Y2 = self.convert ([x2,y2])
        self.file.write ('<line x1="%i" y1="%i" x2="%i" y2="%i"'%(X1,Y1,X2,Y2)+
                         'style="stroke:rgb(0,0,0);stroke-width:1" />\n')

    def initAndGoal (self, xi, yi, xg, yg):
        self.drawCircle ([xi, yi], "black")
        self.drawCircle ([xg, yg], "black")

    def rand (self, x, y):
        self.drawCircle ([x, y], "red")

    def node (self, x, y):
        self.drawCircle ([x, y], "blue")

    def edge (self, x1, y1, x2, y2):
        self.drawLine (x1, y1, x2, y2)
