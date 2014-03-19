#/usr/bin/env python

import time
from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from math import pi

cl = Client ()
cl.robot.loadRobotModel ('hrp2_14', '', '', '')
jn = cl.robot.getJointNames ()

r = ScenePublisher (jn [6:])
q = cl.robot.getCurrentConfig ()

r (q)

q1 = q [::]
q2 = q [::]

q1 [3] = pi/3
q1 [4] = 3*pi/2
q2 [5] = pi - 1e-3

cl.problem.directPath (q1, q2)

pathId = cl.problem.countPaths () - 1
length = cl.problem.pathLength (pathId)

p = PathPlayer (cl, r, pathId)

q1 = [0.0, 0.0, 0.705, 1.0471975511965976, 4.71238898038469, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q2 = [0.0, 0.0, 0.705, 0.0, -0.0, 3.1405926535897932, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
