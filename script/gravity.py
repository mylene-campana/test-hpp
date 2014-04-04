#/usr/bin/env python

from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.tools import PathPlayer
from hrp2 import Robot
from hpp.corbaserver import Client

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot ()
robot.setTranslationBounds (-10, 10, -10, 10, -10, 10)
cl = robot.client
r = ScenePublisher (robot.jointNames [4:])

q_obs=[0, 0, 0, 0, 0, 0, 1] # for the obstacle situation (here, the spaceStation)

q1 = [0.05, 0.01, 0.705, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.8, 0.8, -1.0, -1.0, 0.0, 0.2, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 1.0, -0.8, 0.1, -0.523599, 0.1, -0.3, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.6, 0.1, -0.453786, 0.872665, -0.418879, 0.2, -0.4, 0.0, -0.453786, 0.1, 0.8, 0.0]
r(q1, q_obs)

q2 = [5.05, 4.01, 1.705, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2, 1.0, -0.4, -1.0, 0.0, -0.2, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -1.5, 0.2, 0.1, -0.3, 0.1, 0.1, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, -0.2, 0.6, -0.453786, 0.872665, -0.418879, 0.2, -0.4, 0.0, -0.453786, 0.1, 0.8, 0.0] 
# Unitary quaternion 0.6, 0.8, 0.01, 0.0 but rising an 'innerprod'<1 assertion in : hpp-model/src/joint-configuration.cc => angleBetweenQuaternions function

cl.problem.setInitialConfig (q1)
cl.problem.addGoalConfig (q2)
cl.problem.solve ()

p = PathPlayer (cl, r)
# p(0) or p(1) to play the solution

