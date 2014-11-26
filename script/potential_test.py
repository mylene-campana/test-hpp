#/usr/bin/env python
# Script which goes with potential_description package.
# Load simple 'robot' cylinder and obstacle to test methods.
# "roslaunch potential_description potential_cylinder_point.launch"
# "potential_obstacles_concaves.launch"


from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.simple_robot import Robot
from hpp.corbaserver import Client
import time
import sys
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

robot = Robot ('simple_robot')
cl = robot.client
kRange = 5
robot.setJointBounds('base_joint_xy', [-kRange, kRange, -kRange, kRange])
r = ScenePublisher (robot)

# q = [x, y, z, theta] # (z not considered since planar)
#q1 = [-2.5, 1.0, 1, 0.0]; q2 = [2.5, 1.0, 1, 0.0] # First
#q1 = [-1.8, 0.3, 1, 0.0]; q2 = [1.8, 0.3, 1, 0.0] # First Point
#q1 = [-2.5, 0.0, 1, 0.0]; q2 = [2.5, 0.0, 1, 0.0] # Second
#q1 = [1.8, 0.9, 0.707, 0.707]; q2 = [-1.3, -0.6, 0.707, 0.707] # Third
q1 = [-4, 4, 1, 0]; q2 = [4, -4, 1, 0] # obstS 1
#q1 = [2.4, -4.6, 1.0, 0.0]; q2 = [-0.4, 4.6, 1.0, 0.0] # obstS 2

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance #
#cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle','')
cl.obstacle.loadObstacleModel('potential_description','obstacles_concaves','')
begin=time.time()
cl.problem.solve ()
end=time.time()
print "Solving time: "+str(end-begin)

from trajectory_plot import planarConcObstaclesPlot # case multiple concaves obstacles
planarConcObstaclesPlot(cl, 0, '', 0)

num_log = 7770
from parseLog import parseVisibilityNodes
guardNodes = parseVisibilityNodes (num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/visibility-prm-planner.cc:176: q is a guard node: ')
from trajectory_plot import planarConcObstaclesSpecNodesPlot
planarConcObstaclesSpecNodesPlot(cl, 0, '', 0, guardNodes) # guards in blue


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
from trajectory_plot import planarConcObstaclesPlot # case multiple concaves obstacles
planarConcObstaclesPlot(cl, 0, '', 0)

# Gradients arrows Plot on 2D plan graph (with trajectory) #
from parseLog import parseGrad, parseConfig
num_log = 10435 # TO_FILL, and UPDATE following line numbers
q_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.cc:381: q(x,y): ')
grad_list = parseGrad(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.cc:383: grad(x,y): ')
q_rand_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/diffusing-planner.cc:121: q_rand = ') # diffusingPlanner
#q_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-potential-method.cc:???: q(x,y): ')
#grad_list = parseGrad(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-potential-method.cc:???: grad(x,y): ')
#q_rand_list = parseConfig(num_log, 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/superposed-planner.cc:105: q_rand = ') # superposedPlanner

from trajectory_plot import gradArrowsConcPlot # concave
gradArrowsConcPlot(cl, q_list, grad_list, q_rand_list, 0)

from trajectory_plot import gradArrowsPlot # cylinder
gradArrowsPlot(cl, q_list, grad_list, q_rand_list)

