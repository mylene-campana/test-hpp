#/usr/bin/env python
# Script which goes with gravity.launch, to test Potential Method with 
# HRP2 alone.

from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.hrp2 import Robot
#from hpp.corbaserver.humain import Robot
from hpp.corbaserver import Client
import sys
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'
robot = Robot ('hrp2_14')
#robot = Robot ('humain')
robot.setTranslationBounds (-1, 1, -1, 1, 0.605, 0.805) # rrtTest v1
#robot.setTranslationBounds (-3, 10, -4, 4, -3, 5) # gravity
cl = robot.client
r = ScenePublisher (robot)
p = PathPlayer (cl, r)

q1 = [0.0, 0.0, 0.705, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

cl.problem.lockDof ('base_joint_x', 0, 0, 0) # lock x rotation
cl.problem.lockDof ('base_joint_y', 0, 0, 0) # lock y rotation
cl.problem.lockDof ('base_joint_z', 0.705, 0, 0) # lock z rotation

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
cl.problem.solve ()


""" Case HRP2 is planar (change type in hpp-hrp2/src/robot.py)
q11 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q22 = [0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

cl.problem.lockDof ('base_joint_x', 0, 0, 0) # lock x rotation
cl.problem.lockDof ('base_joint_y', 0, 0, 0) # lock y rotation
cl.problem.lockDof ('base_joint_rz', 0, 0, 0) # lock rz rotation
"""


# Number of Nodes and time information
len(cl.problem.nodes ())
cl.problem.pathLength(0)
cl.problem.pathLength(1)

cl.problem.configAtDistance(0, 0.5)

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
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


# Plot interesting DoF #
from trajectory_plot import dofPlot
num_path = 0 # 0 for non-optim path, 1 for optim path
dofPlot(cl, num_path, 11, 12, 13 ,14) # LArm DoF
dofPlot(cl, num_path, 41, 42, 43 ,45) # RLeg DoF


from trajectory_plot import plannarPlot
# plannarPlot(cl) # must fix distancesToCollision() with min before call



""" # configs q3 q4 from Gravity
q1 = [1.45, 1.05, -0.8, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.8, 1.0, -1.0, -0.85, 0.0, -0.65, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.9, 0.0, -0.6, -0.3, 0.7, -0.4, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.1, -0.15, -0.1, 0.3, -0.418879, 0.0, 0.0, 0.3, -0.8, 0.3, 0.0, 0.0]

q2 = [7.60, -2.41, 0.545, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8, 0.0, -0.4, -0.55, 0.0, -0.6, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.5, -0.2, 0.1, -0.3, 0.1, 0.1, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -0.2, 0.6, -0.1, 1.2, -0.4, 0.2, -0.3, 0.0, -0.4, 0.2, 0.7, 0.0]
"""

# Initial configuration
q3 = [0.0, 0.0, 0.705, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, 0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

gradAtt: 
0,0,0,# trans
0,0,0,# rot (3)
0,0,# chest
0,0,# head
0.0208563,0,-0.00297947,0,0,0,0,# LArm (7)
0,0,0,0,0,# LHand (5)
0,0,0,0,0,0,0,# RArm (7)
0,0,0,0,0,# RHand (5)
0,0,0,0,0,0,# LLeg (6)
0,0,0,0,0,0# RLeg (6)

