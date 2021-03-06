#/usr/bin/env python
# Script which goes with iai_kitchen_in_rviz.launch, to simulate Hrp2 doing some stuff in a kitchen.

from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hrp2 import Robot
from hpp.corbaserver.wholebody_step.client import Client as WsClient

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot ()
robot.setTranslationBounds (-2, 2, -2, 2, 0, 1) # some discontinuities seem to appear in the p(0) solution
cl = robot.client

r = ScenePublisher (robot)
q0 = robot.getInitialConfig ()
r(q0)

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
r(q1,q_obs)
status, q1proj = cl.problem.applyConstraints (q1)

q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

status, q2proj = cl.problem.applyConstraints (q2)

cl.problem.setInitialConfig (q1proj)
cl.problem.addGoalConfig (q2proj)
cl.problem.solve ()

p = PathPlayer (cl, r)
# p(0) or p(1) to play the solution
