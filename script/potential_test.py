#/usr/bin/env python
# Script which goes with potential_description package.
# Load simple 'robot' cylinder and obstacle to test methods.

from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.simple_robot import Robot
from hpp.corbaserver import Client


robot = Robot ('simple_robot')
cl = robot.client

robot.setJointBounds('base_joint_x',[-3, 3])
robot.setJointBounds('base_joint_y',[-3, 3])

r = ScenePublisher (robot)

# q = [x, y, theta]
q1 = [-2.5, 1.0, 0.0]
q2 = [2.5, 1.0, 0.0]
#q2 = [1.8, 0.9, 1.57]

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidanceq
#cl.obstacle.loadObstacleModel('potential_description','obstacle')
cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle')
cl.obstacle.moveObstacle ('obstacle_base', (0,0,0,1,0,0,0)) # necessary ?

cl.problem.solve ()



# Display centered box obstacle
r.addObject('obstacle','obstacle_base')
r(q1)


# Nodes from the roadmap
import time
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
    r (n)
    time.sleep (.2)


## DEBUG commands
cl.obstacle.getObstaclePosition('obstacle_base')
cl.robot.getJointOuterObjects('base_link')
cl.robot.getCurrentConfig()
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(0)
r( cl.problem.configAtDistance(1,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
