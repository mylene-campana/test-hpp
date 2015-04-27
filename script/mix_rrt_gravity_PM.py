#/usr/bin/env python
# Script which goes with gravity.launch, to test Potential Method with 
# HRP2 alone.

from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time
import sys
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'
robot = Robot ('hrp2_14')
cl = robot.client
ps = ProblemSolver (robot)
r = ScenePublisher (robot)
p = PathPlayer (cl, r)
robot.setJointBounds('base_joint_xyz', [-2.1, -0.1, -0.3, 0.6, 0.505, 0.905])

q1 = [-0.2, 0.1, 0.705, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q2 = [-0.2, 0.1, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

# lock hands in closed position
lockedDofs = robot.leftHandClosed ()
for name, value in lockedDofs.iteritems ():
    ps.lockDof (name, value, 0, 0)

lockedDofs = robot.rightHandClosed ()
for name, value in lockedDofs.iteritems ():
    ps.lockDof (name, value, 0, 0)

ps.setInitialConfig (q1)
ps.addGoalConfig (q2)
begin=time.time()
ps.solve ()
end=time.time()
print "Solving time: "+str(end-begin)

len(cl.problem.nodes ())
cl.problem.pathLength(0)

cl.problem.optimizePath (0)

# Load obstacles in HPP JUST for collision avoidance
cl.obstacle.loadObstacleModelCollision('gravity_description','gravity_decor','')
cl.obstacle.loadObstacleModelCollision('gravity_description','emu','')
position_emu= (0,-1,0.2,1,0,0,0)
cl.obstacle.moveObstacle ('emu_base', position_emu)

# Display spaceship environment and Emu guy
r.addObject('spaceShip','obstacle_base') # display
r.moveObject('spaceShip',(0,0,0,1,0,0,0))
r.addObject('emu','emu_base') # display
position_emu= (0,-1,0.2,1,0,0,0)
r.moveObject('emu',position_emu)
r(q1)

# Display Nodes from the roadmap
import time
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
    r (n)
    time.sleep (.2)

## DEBUG commands
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
cl.robot.distancesToCollision()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


# Plot interesting DoF #
from trajectory_plot import dofPlot
num_path = 0 # 0 for non-optim path, 1 for optim path
dofPlot(cl, num_path, 11, 12, 13 ,14) # LArm DoF
dofPlot(cl, num_path, 41, 42, 43 ,45) # RLeg DoF

