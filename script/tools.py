#!/usr/bin/env python
import os
from svg import SvgDrawing

class LogParser (object):
    prefix = ["INFO:/home/florent/devel/airbus/src/hpp-core/src/roadmap.cc:86: Added node: ", "INFO:/home/florent/devel/airbus/src/hpp-core/src/diffusing-planner.cc:48: q_rand = ", "INFO:/home/florent/devel/airbus/src/hpp-core/src/roadmap.cc:160: Added edge between: ", "INFO:/home/florent/devel/airbus/src/hpp-core/src/roadmap.cc:162:                and: "]
    def __init__ (self, pid):
        self.file = file (os.getenv ("DEVEL_DIR") +
                          "/install/var/log/hpp/journal." + str (pid) +
                          ".log")
        self.svg = SvgDrawing (os.getenv ("DEVEL_DIR") +
                               "/install/var/log/hpp/journal." + str (pid) +
                               ".html", 400, 400, 200, 200, 40)
        self.svg.drawRectangle (0,0,1,.5)
        
    def parse (self, nbLines):
        for line, j  in zip (self.file, range (nbLines)):
            for p,i in zip (self.prefix, xrange (100)):
                if line [:len (p)] == p:
                    q = map (float, line [len (p):].split (',')[:-1])
                    if i == 0:
                        # Add node
                        self.svg.node (q [0], q[1])
                    if i == 1:
                        # q_rand
                        self.svg.rand (q [0], q[1])
                    if i == 2:
                        self.nodeFrom = q
                    if i == 3:
                        self.svg.edge (self.nodeFrom [0], self.nodeFrom [1],
                                       q [0], q [1])

