#/usr/bin/env python
# Script which goes with gravity.launch, to simulate Hrp2 in the space with a spaceship from a movie and an emu character from Nasa.gov. See gravity_description package.

from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time
import sys
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'
robot = Robot ('hrp2_14')
#robot.setJointBounds('base_joint_xyz', [-5, 10, -10, 10, -5, 5])
robot.setJointBounds('base_joint_xyz', [-3, 10, -4, 4, -3, 5])
ps = ProblemSolver (robot)
cl = robot.client
r = Viewer (ps)
pp = PathPlayer (cl, r)

# Difficult init config
q3 = [1.45, 1.05, -0.8, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.8, 1.0, -1.0, -0.85, 0.0, -0.65, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.9, 0.0, -0.6, -0.3, 0.7, -0.4, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.1, -0.15, -0.1, 0.3, -0.418879, 0.0, 0.0, 0.3, -0.8, 0.3, 0.0, 0.0]

q2 = [6.55, -2.91, 1.605, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 1.0, -0.4, -1.0, 0.0, -0.2, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.5, -0.2, 0.1, -0.3, 0.1, 0.1, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -0.2, 0.6, -0.453786, 0.872665, -0.418879, 0.2, -0.4, 0.0, -0.453786, 0.1, 0.7, 0.0]

q4 = [7.60, -2.41, 0.545, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8, 0.0, -0.4, -0.55, 0.0, -0.6, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -2.8, 0.0, 0.1, -0.2, -0.1, 0.4, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -0.2, 0.6, -0.1, 1.2, -0.4, 0.2, -0.3, 0.0, -0.4, 0.2, 0.7, 0.0]


# Load obstacles in HPP
cl.obstacle.loadObstacleModel('gravity_description','gravity_decor','') # do not move (position in urdf)
cl.obstacle.loadObstacleModel('gravity_description','emu','') # loaded as an obstacle for now
position_emu= (0,-1,0.2,1,0,0,0)
cl.obstacle.moveObstacle ('emu_base', position_emu)


# Problem
ps.setInitialConfig (q3)
ps.addGoalConfig (q2) # q4 or q2
begin=time.time()
ps.solve ()
end=time.time()
print "Solving time: "+str(end-begin)


# Display spaceship environment and Emu guy
r.addObject('spaceShip','obstacle_base') # display
r.moveObject('spaceShip',(0,0,0,1,0,0,0))
r.addObject('emu','emu_base') # display
position_emu= (0,-1,0.2,1,0,0,0)
r.moveObject('emu',position_emu)
r(q3)

# Nodes from the roadmap
import time
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
        r (n)
        time.sleep (.2)


## DEBUG commands
cl.obstacle.getObstaclePosition('obstacle_base')
cl.robot.getJointOuterObjects('CHEST_JOINT1')
cl.robot.getCurrentConfig()
cl.robot.setCurrentConfig(q3)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(2)
r( cl.problem.configAtDistance(1,5) )
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()

# Initial (q1) and final (q2) simple configurations :
q1 = [0, 0.01, 0.705, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.8, 0.8, -1.0, -1.0, 0.0, 0.2, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.7, -0.8, 0.1, -0.523599, 0.1, -0.3, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.6, 0.1, -0.453786, 0.872665, -0.418879, 0.2, -0.4, 0.0, -0.453786, 0.1, 0.7, 0.0]
q2 = [6.55, -2.91, 1.605, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 1.0, -0.4, -1.0, 0.0, -0.2, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.5, -0.2, 0.1, -0.3, 0.1, 0.1, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -0.2, 0.6, -0.453786, 0.872665, -0.418879, 0.2, -0.4, 0.0, -0.453786, 0.1, 0.7, 0.0]

