#/usr/bin/env python
# Script which goes with gravity.launch, to simulate Hrp2 in the space with a spaceship from a movie and an emu character from Nasa.gov. See gravity_description package.

from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hpp.corbaserver.hrp2 import Robot
import time
from hpp.corbaserver import Client

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'
robot = Robot ('hrp2_14')
robot.setTranslationBounds (-10, 10, -10, 10, -10, 10)
cl = robot.client
r = ScenePublisher (robot.jointNames [4:])

q1 = [0, 0.01, 0.705, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.8, 0.8, -1.0, -1.0, 0.0, 0.2, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.7, -0.8, 0.1, -0.523599, 0.1, -0.3, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.6, 0.1, -0.453786, 0.872665, -0.418879, 0.2, -0.4, 0.0, -0.453786, 0.1, 0.7, 0.0]
q2 = [6.55, -2.91, 1.605, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 1.0, -0.4, -1.0, 0.0, -0.2, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.5, -0.2, 0.1, -0.3, 0.1, 0.1, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -0.2, 0.6, -0.453786, 0.872665, -0.418879, 0.2, -0.4, 0.0, -0.453786, 0.1, 0.7, 0.0]  
q3 = [1.45, 1.05, -0.8, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.8, 1.0, -1.0, -0.85, 0.0, -0.65, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.9, 0.0, -0.6, -0.3, 0.7, -0.4, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.1, -0.15, -0.1, 0.3, -0.418879, 0.0, 0.0, 0.3, -0.8, 0.3, 0.0, 0.0]
q4 = [7.65, -2.41, 0.545, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8, 0.0, -0.4, -0.8, 0.0, -0.7, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -2.7, 0.0, 0.1, -0.2, -0.1, 0.4, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -0.2, 0.6, -0.1, 1.7, -0.4, 0.2, -0.3, 0.0, -0.4, 0.2, 0.7, 0.0]

# Display spaceship environment and Emu guy
r.addObject('spaceShip','obstacle_base') # display
r.moveObject('spaceShip',(0,0,0,1,0,0,0))
r.addObject('emu','emu_base') # display
position_emu= (0,-1,0.2,1,0,0,0)
r.moveObject('emu',position_emu_disp)
r(q1)

cl.problem.setInitialConfig (q3)
cl.problem.addGoalConfig (q4)
p = PathPlayer (cl, r)

# Load obstacles in HPP
cl.obstacle.loadObstacleModel('gravity_description','gravity_decor') # do not move (position in urdf)
cl.obstacle.loadObstacleModel('gravity_description','emu') # loaded as an obstacle for now
cl.obstacle.moveObstacle ('emu_base', position_emu)

cl.problem.solve ()
p(1) #to play the solution


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
cl.robot.setCurrentConfig(q3)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(2)
r( cl.problem.configAtDistance(1,5) )
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()

