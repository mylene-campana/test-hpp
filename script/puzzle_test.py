#/usr/bin/env python
# Script which goes with puzzle_description package.
# Easy way to test planning algo (no internal DoF)

from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
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
q3 = [0.0, 0.0, -0.6, 0.707, 0.0, 0.0, 0.707]

# (Optional if environment ready) Display obstacle
r.addObject('decor','decor_base') # display
r.addObject('decor1','l_decor_one')
r.addObject('decor2','l_decor_two')
r.addObject('decor3','l_decor_three')
r(q1)

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance
cl.obstacle.loadObstacleModel('puzzle_description','decor')

cl.problem.solve ()


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
cl.robot.getCurrentConfig()
cl.robot.setCurrentConfig(q2)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(0)
r( cl.problem.configAtDistance(2,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
