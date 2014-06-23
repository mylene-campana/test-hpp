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

q3 = [0.0, 0.0, 0.705, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, 0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

cl.problem.lockDof ('base_joint_x', 0, 0, 0) # lock x rotation
cl.problem.lockDof ('base_joint_y', 0, 0, 0) # lock y rotation
cl.problem.lockDof ('base_joint_z', 0.705, 0, 0) # lock z rotation
"""
cl.problem.lockDof ('LLEG_JOINT0', 0, 0, 0)
cl.problem.lockDof ('LLEG_JOINT1', 0, 0, 0)
cl.problem.lockDof ('RLEG_JOINT0', 0, 0, 0)
cl.problem.lockDof ('RLEG_JOINT1', 0, 0, 0)
"""
""" PB : assert plus unitaire ? avec hrp2 lors du solve()
cl.problem.lockDof ('base_joint_SO3', 0, 1, 0) # lock x rotation
cl.problem.lockDof ('base_joint_SO3', 0, 2, 1) # lock y rotation"""

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
cl.problem.solve ()


# Number of Nodes and time information
nodes = cl.problem.nodes ()
len(nodes)
cl.problem.pathLength(0)
cl.problem.pathLength(1)


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


#q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0, 0, 0, 0, 0]


""" # configs q3 q4 from Gravity
q1 = [1.45, 1.05, -0.8, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.8, 1.0, -1.0, -0.85, 0.0, -0.65, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.9, 0.0, -0.6, -0.3, 0.7, -0.4, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.1, -0.15, -0.1, 0.3, -0.418879, 0.0, 0.0, 0.3, -0.8, 0.3, 0.0, 0.0]

q2 = [7.60, -2.41, 0.545, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8, 0.0, -0.4, -0.55, 0.0, -0.6, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.5, -0.2, 0.1, -0.3, 0.1, 0.1, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -0.2, 0.6, -0.1, 1.2, -0.4, 0.2, -0.3, 0.0, -0.4, 0.2, 0.7, 0.0]
"""

# premier qSecond : le pied droit bouge vers le gauche (immobile) et entre en collision
r([  ])

# premier qRand : pas terrible vu le but, et ne respecte pas la contrainte !
r([ 1.36075,-0.422468,0.76162,-0.236625,0.381534,0.845472,0.289165,-0.258831,0.784264,-0.349071,0.201546,-1.14187,0.977931,-0.434231,-1.14559,1.45229,1.33657,0.213175,0.341329,-0.562969,0.168026,-0.759793,-0.403873,-2.56676,-0.184331,-1.10254,-1.41841,-1.18889,-1.25627,0.784553,-0.442561,0.0203142,0.532676,0.176934,-0.320393,0.310955,0.154214,-0.743015,2.54576,-0.71167,0.129585,-0.0958904,0.128199,-1.01511,2.33023,-0.730461,-0.0107303 ])

# à voir :
r([ 0,0,0.705,0.999,0,0,0,0,0,0,0,-0.394726,0.00139091,-1.19044,-0.996203,0,0,0.174532,-0.174532,0.174532,-0.174532,0.174532,-0.174532,0.261799,-0.17453,0,-0.523599,0,0,0.174532,-0.174532,0.174532,-0.174532,0.174532,-0.174532,0,0,-0.453786,0.872665,-0.418879,0,-0.0164497,-0.0326049,-0.453786,0.872665,-0.418879,-0.0326049 ])
r([ 0,0,0.705,0.999,0,0,0,-0.000742617,-8.16715e-05,0,0,0.26195,0.174979,-0.000486491,-0.52343,-0.000297672,0.000127295,0.174532,-0.174532,0.174532,-0.174241,0.174241,-0.174241,0.259052,-0.17562,-0.000367359,-0.523071,-0.000402747,0.000446863,0.174532,-0.174532,0.174532,-0.174532,0.174532,-0.174532,0,0,-0.453786,0.872665,-0.418879,0,-0.784398,0.348066,0.229497,1.5552,-0.418897,0.000213946 ])


gradAtt: 
0,0,0,# trans
0,0,0,# rot (3)
0,0,# chest
0,0,# head
0.0208563,0,-0.00297947,0,0,0,0,# LArm (7) => OK pour gradAtt
0,0,0,0,0,# LHand (5)
0,0,0,0,0,0,0,# RArm (7)
0,0,0,0,0,# RHand (5)
0,0,0,0,0,0,# LLeg (6)
0,0,0,0,0,0# RLeg (6)





