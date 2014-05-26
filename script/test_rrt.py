#/usr/bin/env python

from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.wholebody_step.client import Client as WsClient

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot ('hrp2_14')
robot.setTranslationBounds (-3, 3, -3, 3, 0, 1)
cl = robot.client

r = ScenePublisher (robot)
q0 = robot.getInitialConfig ()
r (q0)

# Add constraints
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints ("balance", q0, robot.leftAnkle,
                                           robot.rightAnkle)

p = ProblemSolver (robot)
p.setNumericalConstraints ("balance", ["balance/relative-com",
                                                "balance/relative-orientation",
                                                "balance/relative-position",
                                                "balance/orientation-left-foot",
                                                "balance/position-left-foot"])


# lock hands in closed position
lockedDofs = robot.leftHandClosed ()
for name, value in lockedDofs.iteritems ():
    p.lockDof (name, value, 0, 0)

lockedDofs = robot.rightHandClosed ()
for name, value in lockedDofs.iteritems ():
    p.lockDof (name, value, 0, 0)


q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = p.applyConstraints (q1)
if res [0]:
    q1proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")


q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = p.applyConstraints (q2)
if res [0]:
    q2proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

p.setInitialConfig (q1proj)
p.addGoalConfig (q2proj)
p.solve ()

p = PathPlayer (cl, r)

p (1)

