#/usr/bin/env python

import numpy as np
import matplotlib.pyplot as pl
from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration

# Create simple 2D robot
cfg1 = Configuration (rot = (1,0,0,0,1,0,0,0,1), trs = (0,0,0))
cl = Client ()
cl.robot.createRobot ('robot')
bounds = [-5., 5.]
cl.robot.createJoint ('Tx', 'translation', cfg1, bounds)
cl.robot.setRobotRootJoint ('robot', 'Tx')
cfg2 = Configuration (rot = (0, -1, 0, 1, 0, 0, 0, 0, 1), trs = (0,0,0))
cl.robot.createJoint ('Ty', 'translation', cfg2, bounds)
cl.robot.addJoint ('Tx', 'Ty')
cl.robot.createSphere ("link", 1.)
cl.robot.addObjectToJoint ("Ty", "link", cfg1)
cl.robot.setRobot ('robot')
# Create obstacles
cl.obstacle.createBox ("obst1", 1., 1., 0.2)
#cl.obstacle.addObstacle ("obst1", True, True)

init = [-2., 0.]
goal = [2.,0.]

cl.problem.setInitialConfig (init)
cl.problem.addGoalConfig (goal)

if cl.problem.solve () != 0:
    raise RuntimeError ("failed to solve the problem")

n = cl.problem.numberPaths () -1
if n < 0:
    raise RuntimeError ("Problem solved successfully, but no resulting path")

x = np.array ([])
y = np.array ([])
length = cl.problem.pathLength (n)
dt = 1e-2
nbSamples = int (length/dt + 1)
for i in range (nbSamples):
    t = dt*i
    q = cl.problem.configAtDistance (n, t)
    x = np.append (x, q [0])
    y = np.append (y, q [1])

fig = pl.figure ()
ax = fig.add_subplot ('111')
ax.plot (x,y)
pl.show ()

