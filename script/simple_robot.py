#/usr/bin/env python

from hpp.corbaserver import Client
from hpp_corbaserver.hpp import Configuration

# Create simple 2D robot
cfg1 = Configuration (rot = (1,0,0,0,1,0,0,0,1), trs = (0,0,0))
cl = Client ()
cl.robot.createRobot ('robot')
bounds = [-5., 5., -5., 5., -.01, .01, 1., -1., 1., -1., 1., -1.]
cl.robot.createJoint ('J1', 'freeflyer', cfg1, bounds)
cl.robot.setRobotRootJoint ('robot', 'J1')
cfg2 = Configuration (rot = (0, -1, 0, 1, 0, 0, 0, 0, 1), trs = (0,0,0))
cl.robot.createSphere ("link", 1.)
cl.robot.addObjectToJoint ("J1", "link", cfg1)
cl.robot.setRobot ('robot')
# Create obstacles
cl.obstacle.createBox ("obst1", 1., 1., 0.2)
cl.obstacle.addObstacle ("obst1", True, True)

init = [-2., 0., 0., 0., 0., 0.]
goal = [2.,0., 0., 0., 0., 0.]

cl.problem.setInitialConfig (init)
cl.problem.addGoalConfig (goal)

cl.problem.solve ()
