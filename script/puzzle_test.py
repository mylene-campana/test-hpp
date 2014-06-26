#/usr/bin/env python
# Script which goes with puzzle_description package.
# Easy way to test planning algo (no internal DoF)

from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.puzzle_robot import Robot
from hpp.corbaserver import Client

robot = Robot ('puzzle_robot')
cl = robot.client

robot.setJointBounds('base_joint_x',[-0.9, 0.9])
robot.setJointBounds('base_joint_y',[-0.9, 0.9])
robot.setJointBounds('base_joint_z',[-1.1, 1.1])

r = ScenePublisher (robot)

q1 = [0.0, 0.0, 0.6, 1.0, 0.0, 0.0, 0.0]
q2 = [0.0, 0.0, -0.6, 1.0, 0.0, 0.0, 0.0] 
#q3 = [0.0, 0.0, -0.6, 0.707, 0.0, 0.0, 0.707]

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance
cl.obstacle.loadObstacleModel('puzzle_description','decor')
#cl.obstacle.loadObstacleModel('puzzle_description','decor_easy')

cl.problem.solve ()


# (Optional if environment ready) Display obstacle
r.addObject('decor','decor_base') # display
r.addObject('decor1','l_decor_one')
r.addObject('decor2','l_decor_two')
r.addObject('decor3','l_decor_three')
r(q1)


len(cl.problem.nodes ())
cl.problem.pathLength(0)
cl.problem.pathLength(1)

# Nodes from the roadmap
import time
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
    r (n)
    time.sleep (.2)


## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
cl.robot.getJointOuterObjects('j_object_one')
cl.robot.getJointOuterObjects('j_object_two')
cl.robot.getJointOuterObjects('j_object_three')
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
cl.robot.distancesToCollision()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])
r( cl.problem.configAtDistance(2,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()

from numpy import *
argmin(cl.robot.distancesToCollision()[0])

# Plot interesting DoF #
from trajectory_plot import dofPlot
num_path = 0 # 0 for non-optim path, 1 for optim path
dofPlot(cl, num_path, 0, 1, 2 ,3)


# pb calcul distance entre Bodies successifs
>>> cl.robot.distancesToCollision()[1]
>>> cl.robot.distancesToCollision()[2]

