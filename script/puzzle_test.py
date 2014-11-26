#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.puzzle_robot import Robot
from hpp.corbaserver import Client
import time
import sys
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

robot = Robot ('puzzle_robot')
cl = robot.client
robot.setJointBounds('base_joint_xyz', [-0.9, 0.9, -0.9, 0.9, -1.1, 1.1])
r = ScenePublisher (robot)

q1 = [0.0, 0.0, 0.6, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.6, 1.0, 0.0, 0.0, 0.0]
#q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
#q1 = [0.5, 0.3, 0.8, 0.7071067812, 0.0, 0.7071067812, 0.0]; q2 = [0.0, -0.6, -0.5, 1.0, 0.0, 0.0, 0.0]
r(q1)

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance
cl.obstacle.loadObstacleModel('puzzle_description','decor_very_easy','')
#cl.obstacle.loadObstacleModel('puzzle_description','decor_easy','')
#cl.obstacle.loadObstacleModel('puzzle_description','decor','')

begin=time.time()
cl.problem.solve ()
end=time.time()
print "Solving time: "+str(end-begin)

len(cl.problem.nodes ())
cl.problem.pathLength(0)

from trajectory_plot import xyzPlot
pathNum = 0
xyzPlot (cl, pathNum)

# (Optional if environment ready) Display obstacle
r.addObject('decor','decor_base') # display
r.addObject('decor1','l_decor_one')
r.addObject('decor2','l_decor_two')
r.addObject('decor3','l_decor_three')
r(q1)

# Nodes from the roadmap
import time
for n in  cl.problem.nodes ():
    r (n)
    time.sleep (.3)


## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
cl.robot.getJointOuterObjects('j_object_one')
cl.robot.getJointOuterObjects('j_object_two')
cl.robot.getJointOuterObjects('j_object_three')
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
cl.robot.distancesToCollision()
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])

