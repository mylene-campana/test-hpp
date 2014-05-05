#/usr/bin/env python

from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver.wholebody_step.client import Client as WsClient

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot ('hrp2_14')
robot.setTranslationBounds (-3, 3, -3, 3, 0, 1)
cl = robot.client

r = ScenePublisher (robot.jointNames [4:])
q0 = robot.getInitialConfig ()
r (q0)

# Add constraints
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints (q0)
# lock hands in closed position
lockedDofs = robot.leftHandClosed ()
for index, value in lockedDofs:
    cl.problem.lockDof (index, value)

lockedDofs = robot.rightHandClosed ()
for index, value in lockedDofs:
    cl.problem.lockDof (index, value)


q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q1proj = cl.problem.applyConstraints (q1)

q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q2proj = cl.problem.applyConstraints (q2)

cl.problem.setInitialConfig (q1proj)
cl.problem.addGoalConfig (q2proj)
cl.problem.solve ()

