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
#q1 = [-2.5, 1.0, 0.0]
#q2 = [2.5, 1.0, 0.0]
q1 = [-2.5, 0.0, 0.0]
q2 = [2.5, 0.0, 0.0]
#q1 = [1.8, 0.9, 1.57]
#q2 = [-1.3, -0.6, 1.57]

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance
#cl.obstacle.loadObstacleModel('potential_description','obstacle') # box
cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle')
cl.obstacle.moveObstacle ('obstacle_base', (0,0,0,1,0,0,0))

cl.problem.solve ()


len(cl.problem.nodes ())
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
from numpy import *
argmin(cl.robot.distancesToCollision()[0])
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


# Plot Trajectory, x, y, dist_obst #
from trajectory_plot import plannarPlot
plannarPlot(cl)

# Gradients (t) Plot from Log Parser # -only require Log files
from parseLog import parseGrad
from trajectory_plot import gradientPlot
gradAtt = parseGrad(5901, \
'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.hh:160: gradAtt: ')
gradRep = parseGrad(5901, \
'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.hh:153: gradRep: ')
gradientPlot(gradAtt, gradRep)

# Gradients arrows Plot on 2D plan graph (with trajectory) #
from parseLog import parseGrad
from trajectory_plot import gradArrowsPlot
num_log = 14760 # TO_FILL
q_list = parseGrad(num_log, \
'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.hh:470: q(x,y): ')
grad_list = parseGrad(num_log, \
'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.hh:471: grad(x,y): ')
gradArrowsPlot(cl, q_list, grad_list)
# arrow size *2 for more visibility !

