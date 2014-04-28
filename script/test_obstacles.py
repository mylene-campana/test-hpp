#/usr/bin/env python

from hpp_corbaserver.hpp import Configuration
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
q_obs=[0, 0, 0, 1, 0, 0, 0]
#r(q0,q_obs)

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
p = PathPlayer (cl, r)

# New way to add obstacles to hpp via the urdf model !
cl.obstacle.loadObstacleModel('boxes_description','boxes')
position_g= Configuration (trs=(0,0,1) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ('obstacle_base', position_g)
position_b= Configuration (trs=(0,3,1) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ('obstacle_base_two', position_b)
position_r= Configuration (trs=(0,-3,1) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ('obstacle_base_three', position_r)
cl.problem.solve ()

cl.obstacle.getObstaclePosition('obstacle_base_two') # DEBUG
cl.robot.getJointOuterObjects('CHEST_JOINT1') # DEBUG


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


# Nodes from the roadmap
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
    r (n)
    time.sleep (.2)


# Old creating geometries to add to hpp
cl.obstacle.createBox ("obst1", 1., 2., 2.) # green box
cl.obstacle.addObstacle ("obst1", True, True)
cfg_obs1= Configuration (trs=(0,0,0) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ("obst1", cfg_obs1)

cl.obstacle.createBox ("obst2", 1., 2., 2.) # blue box
cl.obstacle.addObstacle ("obst2", True, True) 
cfg_obs2= Configuration (trs=(0,3,0) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ("obst2", cfg_obs2)

cl.obstacle.createBox ("obst3", 1., 2., 2.) # red box
cl.obstacle.addObstacle ("obst3", True, True) 
cfg_obs3= Configuration (trs=(0,-3.,0) , quat=(1,0,0,0))
cl.obstacle.moveObstacle ("obst3", cfg_obs3)


# Fun position
[-0.05143255675322164, 0.05672844080847696, 0.5811141468403341, -0.1795415654094085, 0.05565995693322502, 1.0476272486668958, 0.10363982764531976, -1.5257791953940159, -0.3591582896128643, -0.4370686943894689, 0.04260775384531124, 0.2310271421968017, 0.521287617017005, -0.4918950089993315, -0.3134599350834638, -0.027680119485486952, 0.03246346098321952, 0.002438359134500878, -0.3856595235351046, -1.4389904886275202, 1.179903782468734, -0.6679912960032259, 0.13166469273996773, 0.12640639104539758, 0.5508529432400333, -0.43857371250089633, -0.4590377121680009, -0.3567152705827428, 0.16068888686272273, -0.119854847284512, -1.9213137424874347, 0.31939552783102176, -0.662318853366131, 0.24989100390577623, -0.4645924882709224, -0.5079395233899916, -0.44678023778923504, 1.8830650878119608, -0.4271416376238236, 0.45215542393888575]
