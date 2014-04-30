#/usr/bin/env python

from hpp_corbaserver.hpp import Configuration
from hpp_ros import ScenePublisher
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import Client

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix = '_capsule'
robot = Robot ('hrp2_14')
cl = robot.client
r = ScenePublisher (robot.jointNames [4:])
q0 = robot.getInitialConfig ()
r(q0)
