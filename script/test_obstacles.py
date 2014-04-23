#/usr/bin/env python

from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hrp2 import Robot
from hpp.corbaserver.wholebody_step.client import Client as WsClient

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot () # hrp2 with capsules for collision tests
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
status, q1proj = cl.problem.applyConstraints (q1)

q2=[4,0,0,1,0,0,0,0.0, 0.0, 0.0, 0.0, 0.261799, 0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
status, q2proj = cl.problem.applyConstraints (q2)

# Build the problem
cl.problem.setInitialConfig (q1proj)
cl.problem.addGoalConfig (q2proj)
cl.problem.solve ()

p = PathPlayer (cl, r)
#p(1)


# Create geometries to add to hpp
cl.obstacle.createBox ("obst1", 1., 2., 2.) # green box
cl.obstacle.addObstacle ("obst1", True, True)
cfg_obs1= Configuration (trs=(0,0,0) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ("obst1", cfg_obs1)
cl.problem.solve ()
#p(3)

cl.obstacle.createBox ("obst2", 1., 2., 2.) # blue box
cl.obstacle.addObstacle ("obst2", True, True) 
cfg_obs2= Configuration (trs=(0,3,0) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ("obst2", cfg_obs2)
cl.problem.solve ()
#p(5)

cl.obstacle.createBox ("obst3", 1., 2., 2.) # red box
cl.obstacle.addObstacle ("obst3", True, True) 
cfg_obs3= Configuration (trs=(0,-3.,0) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ("obst3", cfg_obs3)
cl.problem.solve ()
#p(7)

# New way to display obstacles in RViz
r.addObject('boxes','obstacle_base') # green box
position_g= Configuration (trs=(0,0,0) , quat=(1,0,0,0))
r.moveObject('boxes',position_g)
r(q1)

r.addObject('blue_box','obstacle_base_two') # blue box
position_b= Configuration (trs=(0,3,0) , quat=(1,0,0,0))
r.moveObject('blue_box',position_b)
r(q1)

r.addObject('red_box','obstacle_base_three') # red box
position_r= Configuration (trs=(0,-3,0) , quat=(1,0,0,0))
r.moveObject('red_box',position_r)
r(q1)


# New way to add obstacles to hpp via the urdf model !
cl.obstacle.loadObstacleModel('boxes_description','boxes')
position_b= Configuration (trs=(0,3,1) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ('obstacle_base_two', position_b)
position_r= Configuration (trs=(0,-3,1) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ('obstacle_base_three', position_r)

cl.obstacle.getObstaclePosition('obstacle_base_two') # DEBUG
cl.robot.getJointOuterObjects('CHEST_JOINT1') # DEBUG
