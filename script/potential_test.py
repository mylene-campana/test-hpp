#/usr/bin/env python
# Script which goes with potential_description package.
# Load simple 'robot' cylinder and obstacle to test methods.

from hpp_ros import ScenePublisher
from hpp_ros import PathPlayer
from hpp.corbaserver.simple_robot import Robot
from hpp.corbaserver import Client


robot = Robot ('simple_robot')
cl = robot.client

robot.setJointBounds('base_joint_x',[-3, 3])
robot.setJointBounds('base_joint_y',[-3, 3])

r = ScenePublisher (robot)

# q = [x, y, theta]
q1 = [-2.5, 1.0, 0.0] # first test : 5.78s < SMS
q2 = [2.5, 1.0, 0.0]
#q1 = [-2.5, 0.0, 0.0] # second test : 17s à battre
#q2 = [2.5, 0.0, 0.0]
#q1 = [1.8, 0.9, 1.57] # third test : 18.45s à battre
#q2 = [-1.3, -0.6, 1.57]

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
p = PathPlayer (cl, r)

# Load box obstacle in HPP for collision avoidance
#cl.obstacle.loadObstacleModel('potential_description','obstacle') # box
cl.obstacle.loadObstacleModel('potential_description','cylinder_obstacle')
cl.obstacle.moveObstacle ('obstacle_base', (0,0,0,1,0,0,0))

cl.problem.solve ()


nodes = cl.problem.nodes ()
len(nodes)
cl.problem.pathLength(0)
cl.problem.pathLength(1)


# Display centered box obstacle
#r.addObject('obstacle','obstacle_base') # box
r.addObject('cylinder_obstacle','obstacle_base')
r(q1)


# Nodes from the roadmap
import time
nodes = cl.problem.nodes ()
len(nodes)
for n in  nodes:
    r (n)
    time.sleep (.7)


## DEBUG commands
cl.robot.setCurrentConfig(q2)
cl.robot.collisionTest()
cl.robot.distancesToCollision()
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


# Ploting stuff ############################
import matplotlib.pyplot as plt
import numpy as np
dt = 0.1
t_vec = np.arange(0., cl.problem.pathLength(0), dt) # all

Tvec = t_vec [::]
plt.subplot(221)
circle_obst=plt.Circle((0,0),.4,color='r')
plt.gcf().gca().add_artist(circle_obst)
for t in Tvec:
    plt.plot([cl.problem.configAtDistance(0, t)[1], \
                 cl.problem.configAtDistance(0, t+dt)[1]], \
                 [cl.problem.configAtDistance(0, t)[0], \
                 cl.problem.configAtDistance(0, t+dt)[0]], 'k')

plt.axis([-3, 3, -3, 3]) #Ymin Ymax Xmin Xmax
plt.xlabel('y')
plt.ylabel('x')
plt.title('trajectory')
plt.grid()
plt.gca().invert_xaxis()
plt.text(.5, -.5, r'obstacle')

plt.subplot(222)
for t in Tvec:
    plt.plot(t, cl.problem.configAtDistance(0, t)[0], 'ro')

plt.axis([min(Tvec),max(Tvec),-3,3]) #Xmin Xmax Ymin Ymax
plt.xlabel('t')
plt.ylabel('x')
plt.grid()

plt.subplot(223)
for t in Tvec:
    plt.plot(t, cl.problem.configAtDistance(0, t)[1], 'ro')

plt.axis([min(Tvec),max(Tvec),-3,3]) #Xmin Xmax Ymin Ymax
plt.xlabel('t')
plt.ylabel('y')
plt.grid()

plt.subplot(224)
for t in Tvec:
    plt.plot(t, cl.problem.configAtDistance(0, t)[2], 'ro')

plt.axis([min(Tvec),max(Tvec),-3.14,3.14]) #Xmin Xmax Ymin Ymax
plt.xlabel('t')
plt.ylabel('theta')
plt.grid()

plt.show() # works only once
#fig.savefig('plotcircles.png') # to save picture
