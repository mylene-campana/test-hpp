#/usr/bin/env python

from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.wholebody_step.client import Client as WsClient
from hpp.corbaserver.wholebody_step import Problem

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot ('hrp2_14')
robot.setJointBounds ("base_joint_xyz", [-3, 3, -3, 3, 0, 1])
ps = ProblemSolver (robot)

cl = robot.client

r = Viewer (ps)
q0 = robot.getInitialConfig ()
r (q0)

# Add constraints
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints ("balance", q0, robot.leftAnkle,
                                           robot.rightAnkle, "",
                                           Problem.SLIDING)

ps.setNumericalConstraints ("balance", ["balance/relative-com",
                                                "balance/relative-orientation",
                                                "balance/relative-position",
                                                "balance/orientation-left-foot",
                                                "balance/position-left-foot"])

# lock hands in closed position
lockedDofs = robot.leftHandClosed ()
for name, value in lockedDofs.iteritems ():
    ps.lockJoint (name, value)

lockedDofs = robot.rightHandClosed ()
for name, value in lockedDofs.iteritems ():
    ps.lockJoint (name, value)


q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q1)
if res [0]:
    q1proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")


q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q2)
if res [0]:
    q2proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

ps.setInitialConfig (q1proj)
ps.addGoalConfig (q2proj)
ps.addPathOptimizer ("RandomShortcut")
ps.solve ()

pp = PathPlayer (cl, r)
#pp (1)
