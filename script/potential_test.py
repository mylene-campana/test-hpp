#/usr/bin/env python
# Script which goes with potential_description package.
# Load simple 'robot' cylinder and obstacle to test methods.

from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.simple_robot import Robot
from hpp.corbaserver import Client
import sys
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')


robot = Robot ('simple_robot')
cl = robot.client

robot.setJointBounds('base_joint_x',[-3, 3])
robot.setJointBounds('base_joint_y',[-3, 3])

r = ScenePublisher (robot)

# q = [x, y, theta]
q1 = [-2.5, 1.0, 0.0] # first test : 5.78s < SMS
q2 = [2.5, 1.0, 0.0]
#q1 = [-2.5, 0.0, 0.0] # second test : 17s à battre
#q2 = [2.5, 0.0, 0.0]
#q1 = [1.8, 0.9, 1.57] # third test : 18.45s à battre
#q2 = [-1.3, -0.6, 1.57]

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance
#cl.obstacle.loadObstacleModel('potential_description','obstacle') # box
cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle')
cl.obstacle.moveObstacle ('obstacle_base', (0,0,0,1,0,0,0))

cl.problem.solve ()


nodes = cl.problem.nodes ()
len(nodes)
cl.problem.pathLength(0)
cl.problem.pathLength(1)


# Display centered box obstacle
#r.addObject('obstacle','obstacle_base') # box
r.addObject('cylinder_obstacle','obstacle_base')
r(q1)


# Nodes from the roadmap
import time
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
    r (n)
    time.sleep (.7)


## DEBUG commands
cl.robot.setCurrentConfig(q2)
cl.robot.collisionTest()
cl.robot.distancesToCollision()
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


# Plot Trajectory, x, y, dist_obst #
from trajectory_plot import plannarPlot
plannarPlot(cl)

# Gradient Plot from Log Parser #
from parseLog import parseGrad
gradAtt = parseGrad(10160, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.hh:156: gradAtt: ')
gradRep = parseGrad(10160, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.hh:149: gradRep: ')

from trajectory_plot import gradientPlot
gradientPlot(gradAtt, gradRep)

