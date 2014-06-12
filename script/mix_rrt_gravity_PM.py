#/usr/bin/env python
# Script which goes with gravity.launch, to test Potential Method with 
# HRP2 alone.

from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import Client

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'
robot = Robot ('hrp2_14')
robot.setTranslationBounds (-2, 2, -2, 2, 0.605, 0.805) # rrt
#robot.setTranslationBounds (-3, 10, -4, 4, -3, 5) # gravity
cl = robot.client
r = ScenePublisher (robot)
p = PathPlayer (cl, r)

q1 = [0.0, 0.0, 0.705, 1.0, 0., 0., 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q2 = [0.0, 0.0, 0.705, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
cl.problem.solve ()

""" # configs q3 q4 de gravity
q1 = [1.45, 1.05, -0.8, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.8, 1.0, -1.0, -0.85, 0.0, -0.65, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.9, 0.0, -0.6, -0.3, 0.7, -0.4, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.1, -0.15, -0.1, 0.3, -0.418879, 0.0, 0.0, 0.3, -0.8, 0.3, 0.0, 0.0]

q2 = [7.60, -2.41, 0.545, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8, 0.0, -0.4, -0.55, 0.0, -0.6, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.5, -0.2, 0.1, -0.3, 0.1, 0.1, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -0.2, 0.6, -0.1, 1.2, -0.4, 0.2, -0.3, 0.0, -0.4, 0.2, 0.7, 0.0]
"""

# Nodes from the roadmap
nodes = cl.problem.nodes ()
len(nodes)
cl.problem.pathLength(0)
cl.problem.pathLength(1)

## DEBUG commands
cl.obstacle.getObstaclePosition('obstacle_base')
cl.robot.getJointOuterObjects('CHEST_JOINT1')
cl.robot.getCurrentConfig()
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(0)
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


# Ploting stuff ############################
import matplotlib.pyplot as plt
import numpy as np
t_vec = np.arange(0., 8., 0.2)
for t in t_vec:
    #x = configAtSize(0, t)
    plt.plot(cl.robot.configAtDistance(0, t)[0], cl.robot.configAtDistance(0, t)[1], 'ro')

plt.axis([-2, 2, -2, 2]) #Xmin Xmax Ymin Ymax
plt.xlabel('x')
plt.ylabel('y')
plt.show() # works only once


for t in t_vec:
    plt.plot(t, cl.problem.configAtDistance(0, t)[1], 'ro')


# Plot interesting DoF (here RLeg) ###
import matplotlib.pyplot as plt
import numpy as np
dt = 0.1
t_vec = np.arange(0., 8., dt) # all
t_ssvec = np.arange(4.2, 6., dt)

Tvec = t_vec [::]
plt.subplot(221)
for t in Tvec:
    #plt.plot(t, cl.problem.configAtDistance(0, t)[41], 'ro')
    plt.plot([t, t+dt], [cl.problem.configAtDistance(0, t)[41], \
                             cl.problem.configAtDistance(0, t+dt)[41]], 'k-')

plt.axis([min(Tvec),max(Tvec),-2,2]) #Xmin Xmax Ymin Ymax
plt.xlabel('t')
plt.ylabel('theta41')
plt.grid()

plt.subplot(222)
for t in Tvec:
    #plt.plot(t, cl.problem.configAtDistance(0, t)[42], 'ro')
    plt.plot([t, t+dt], [cl.problem.configAtDistance(0, t)[42], \
                             cl.problem.configAtDistance(0, t+dt)[42]], 'k-')

plt.axis([min(Tvec),max(Tvec),-2,2]) #Xmin Xmax Ymin Ymax
plt.xlabel('t')
plt.ylabel('theta42')
plt.grid()

plt.subplot(223)
for t in Tvec:
    #plt.plot(t, cl.problem.configAtDistance(0, t)[43], 'ro')
    plt.plot([t, t+dt], [cl.problem.configAtDistance(0, t)[43], \
                             cl.problem.configAtDistance(0, t+dt)[43]], 'k-')

plt.axis([min(Tvec),max(Tvec),-2,2]) #Xmin Xmax Ymin Ymax
plt.xlabel('t')
plt.ylabel('theta43')
plt.grid()

plt.subplot(224)
for t in Tvec:
    #plt.plot(t, cl.problem.configAtDistance(0, t)[45], 'ro')
    plt.plot([t, t+dt], [cl.problem.configAtDistance(0, t)[45], \
                             cl.problem.configAtDistance(0, t+dt)[45]], 'k-')

plt.axis([min(Tvec),max(Tvec),-2,2]) #Xmin Xmax Ymin Ymax
plt.xlabel('t')
plt.ylabel('theta45')
plt.grid()

plt.show() # works only once
