#/usr/bin/env python
# Script which goes with potential_description package.
# Load simple 'robot' cylinder and obstacle to test methods.
# "roslaunch potential_description potential_cylinder_point.launch"
# "roslaunch potential_description potential_obstacles.launch"


from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.simple_robot import Robot
from hpp.corbaserver import Client
import sys
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')


robot = Robot ('simple_robot')
cl = robot.client

kRange = 5
robot.setJointBounds('base_joint_x',[-kRange, kRange])
robot.setJointBounds('base_joint_y',[-kRange, kRange])

r = ScenePublisher (robot)

# q = [x, y, theta] #
#q1 = [-2.5, 1.0, 0.0]; q2 = [2.5, 1.0, 0.0] # First
#q1 = [-2.5, 0.0, 0.0]; q2 = [2.5, 0.0, 0.0] # Second
#q1 = [1.8, 0.9, 1.57]; q2 = [-1.3, -0.6, 1.57] # Third
#q1 = [-4, 4, 0]; q2 = [4, -4, 0] # obstS 1
q1 = [2.4, -4.6, 0]; q2 = [-0.4, 4.6, 0] # obstS 2

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance #
#cl.obstacle.loadObstacleModel('potential_description','obstacle') # box
#cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle')
#cl.obstacle.loadObstacleModel('potential_description','obstacles')
cl.obstacle.loadObstacleModel('potential_description','obstacles_concaves')

cl.problem.solve ()

from trajectory_plot import planarConcObstaclesPlot # case multiple concaves obstacles
planarConcObstaclesPlot(cl, 0, '', 0)

len(cl.problem.nodes ())
cl.problem.pathLength(0)


# Display obstacle(s) from launch file
r.addObject('obst0','obstacle_base')
r.addObject('obst1','l_object_one'); r.addObject('obst2','l_object_two')
r.addObject('obst3','l_object_three'); r.addObject('obst4','l_object_four')
r.addObject('obst5','l_object_five'); r.addObject('obst6','l_object_six')
r.addObject('obst7','l_object_seven'); r.addObject('obst8','l_object_eight')
r.addObject('obst9','l_object_nine'); r.addObject('obst10','l_object_ten')
r.addObject('obst11','l_object_eleven'); r.addObject('obst12','l_object_twelve')
r.addObject('obst1b','l_object_one_bis');
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
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


# Plot Trajectory, x, y, dist_obst #
from trajectory_plot import plannarPlot 
plannarPlot(cl)
from trajectory_plot import planarObstaclesPlot # case multiple obstacles
planarObstaclesPlot(cl, 0, '', 0)
from trajectory_plot import planarConcObstaclesPlot # case multiple concaves obstacles
planarConcObstaclesPlot(cl, 0, '', 0)

# Gradients arrows Plot on 2D plan graph (with trajectory) #
from parseLog import parseGrad, parseConfig
from trajectory_plot import gradArrowsPlot
num_log = 13318 # TO_FILL, and UPDATE following line numbers
#q_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.hh:502: q(x,y): ')
#grad_list = parseGrad(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.hh:506: grad(x,y): ')
q_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-potential-method.cc:411: q(x,y): ')
q_rand_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-planner.cc:120: q_rand = ') # may be empty
grad_list = parseGrad(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-potential-method.cc:413: grad(x,y): ')
gradArrowsPlot(cl, q_list, grad_list, q_rand_list)


# q_rands de obstacleS2 :
qr1 = [-2.31801,3.92034,0.277004] # fail 1600 iters
qr2 = [-0.105368,-2.76147,-2.16115] # dij_ < 0
qr3 = [-1.56923,1.30571,-0.298299] # fail 1600 iters
qr4 = [-0.768075,-1.25183,2.28836] # fail 1600 iters
# 1799 shoots in collision 
qr5 = [-0.0354639,-3.88197,1.7024] # fail 1600 iters
qr6 = [3.55087,0.252905,-2.67904]
