#/usr/bin/env python
# Script which goes with puzzle_description package.
# Easy way to test planning algo (no internal DoF)

from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver import Robot
from hpp.corbaserver import Client

cl = Client ()
cl.robot.loadRobotModel ('object', 'freeflyer', 'puzzle_description', 'object','', '')


cl.robot.setJointBounds('base_joint_x',[-1, 1])
cl.robot.setJointBounds('base_joint_y',[-1, 1])
cl.robot.setJointBounds('base_joint_z',[-1, 1])

jn = cl.robot.getJointNames ()
r = ScenePublisher (jn [4:])

q1 = [0.0, 0.0, 0.6, 1.0, 0.0, 0.0, 0.0]
q2 = [0.0, 0.0, -0.6, 1.0, 0.0, 0.0, 0.0] 
q3 = [0.0, 0.0, -0.6, 0.5, 0.0, 0.0, 0.5] 


# Display centered box obstacle
r.addObject('decor','decor_base') # display
r(q1)
r.addObject('decor','l_decor_one')
r(q1)
r.addObject('decor','l_decor_two')
r(q1)
r.addObject('decor','l_decor_three')
r(q1)

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance
cl.obstacle.loadObstacleModel('puzzle_description','decor')

cl.problem.solve ()
# PB : ça a tendance à traverser l'obstacle (genre de saut dans RRT) mais on peut voir la config où ça chie
# pourtant c'est faisable !!

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
cl.robot.getCurrentConfig()
cl.robot.setCurrentConfig(q2)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(0)
r( cl.problem.configAtDistance(1,5) )
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()
