#/usr/bin/env python
# Script which goes with potential_description package.
# Load simple 'robot' cylinder and obstacle to test methods.

from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver import Robot
from hpp.corbaserver import Client

cl = Client ()
cl.robot.loadRobotModel ('object', 'freeflyer', 'potential_description', 'object','', '')


cl.robot.setJointBounds('base_joint_x',[-3, 3])
cl.robot.setJointBounds('base_joint_y',[-3, 3])
cl.robot.setJointBounds('base_joint_z',[-0.01, 0.01])
cl.problem.lockDof ('base_joint_SO3', 0, 1, 0) # lock x rotation
cl.problem.lockDof ('base_joint_SO3', 0, 2, 1) # lock y rotation

jn = cl.robot.getJointNames ()
r = ScenePublisher (jn [4:])

q1 = [-2.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
q2 = [2.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] 

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

cl.problem.solve ()

# Load box obstacle in HPP for collision avoidance
cl.obstacle.loadObstacleModel('potential_description','obstacle')


# Display centered box obstacle
r.addObject('obstacle','obstacle_base') # display
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
cl.robot.setCurrentConfig(q4)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(0)
r( cl.problem.configAtDistance(1,5) )
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()
