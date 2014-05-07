#/usr/bin/env python
# Script which goes with boxes.launch, to simulate Hrp2 sliding on the floor with three boxes as obstacles (see boxes_description package).

from hpp.corbaserver import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.hrp2 import Robot
import time
from hpp.corbaserver.wholebody_step.client import Client as WsClient

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix = '_capsule'
robot = Robot ('hrp2_14')
robot.setTranslationBounds (-4, 4, -4, 4, 0, 1)
cl = robot.client

r = ScenePublisher (robot.jointNames [4:])
q0 = robot.getInitialConfig ()
r(q0)

# Add constraints
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints (q0)
# lock hands in closed position
lockedDofs = robot.leftHandClosed ()
for index, value in lockedDofs:
    cl.problem.lockDof (index, value)

lockedDofs = robot.rightHandClosed ()
for index, value in lockedDofs:
    cl.problem.lockDof (index, value)

q1=[-4,0,0,1,0,0,0, 0.0, 0.0, 0.0, 0.0, 0.261799, 0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
q1proj = cl.problem.applyConstraints (q1)

q2=[4,0,0,1,0,0,0,0.0, 0.0, 0.0, 0.0, 0.261799, 0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
q2proj = cl.problem.applyConstraints (q2)

# Build the problem
cl.problem.setInitialConfig (q1proj)
cl.problem.addGoalConfig (q2proj)
p = PathPlayer (cl, r)
cl.problem.solve ()

# Add obstacles to hpp via the urdf model
cl.obstacle.loadObstacleModel('boxes_description','boxes')
position_g= Configuration (trs=(0,0,1) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ('obstacle_base', position_g)
position_b= Configuration (trs=(0,3,1) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ('obstacle_base_two', position_b)
position_r= Configuration (trs=(0,-3,1) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ('obstacle_base_three', position_r)
cl.problem.solve ()


# Display obstacles in RViz
r.addObject('boxes','obstacle_base') # green box
position_g= Configuration (trs=(0,0,0) , quat=(1,0,0,0))
r.moveObject('boxes',position_g)

r.addObject('blue_box','obstacle_base_two') # blue box
position_b= Configuration (trs=(0,3,0) , quat=(1,0,0,0))
r.moveObject('blue_box',position_b)

r.addObject('red_box','obstacle_base_three') # red box
position_r= Configuration (trs=(0,-3,0) , quat=(1,0,0,0))
r.moveObject('red_box',position_r)
r(q1)


# Nodes from the roadmap
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
    r (n)
    time.sleep (.2)


## DEBUG commands
cl.obstacle.getObstaclePosition('obstacle_base')
cl.robot.getJointOuterObjects('CHEST_JOINT1')
cl.robot.getCurrentConfig()
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(1)
r( cl.problem.configAtDistance(1,5) )
cl.problem.clearRoadmap ()
cl.problem.optimizePath (2)
cl.problem.directPath(q1,q2)

