#/usr/bin/env python

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

# Display furnitures in RViz
k=2.3
kk=k-1
r.addObject('lamp','lamp_base')
lamp_pos= Configuration (trs=(0-0.3,-k,0) , quat=(1,0,0,0))
r.moveObject('lamp',lamp_pos)
r.addObject('armchair','armchair_base')
armchair_pos= Configuration (trs=(kk,kk,0) , quat=(0.5,0,0,-0.5))
r.moveObject('armchair',armchair_pos)
r.addObject('pc','pc_base')
pc_pos= Configuration (trs=(k,-k+0.4,1.04) , quat=(0.5,0,0,-0.5))
r.moveObject('pc',pc_pos)
r(q0)
r.addObject('desk','desk_base')
desk_pos= Configuration (trs=(k,-k,0) , quat=(0.5,0,0,-0.5))
r.moveObject('desk',desk_pos)
r.addObject('chair','chair_base')
chair_pos= Configuration (trs=(kk,-k+0.4,0) , quat=(0.5,0,0,0.5))
r.moveObject('chair',chair_pos)
r.addObject('table','table_base')
table_pos= Configuration (trs=(0,kk,0) , quat=(1,0,0,0))
r.moveObject('table',table_pos)
r.addObject('commode','commode_base')
commode_pos= Configuration (trs=(-k-0.2,-k+0.2,0) , quat=(0.5,0,0,0.5))
r.moveObject('commode',commode_pos)
r.addObject('books','books_base')
books_pos= Configuration (trs=(-k+0.1,-k,1) , quat=(0.5,0,0,0.5))
r.moveObject('books',books_pos)
r.addObject('deskLamp','deskLamp_base')
deskLamp_pos= Configuration (trs=(k+0.3,-k+0.2,0.98) , quat=(0.5,0,0,-0.5))
r.moveObject('deskLamp',deskLamp_pos)
r.addObject('beer','beer_base')
beer_pos= Configuration (trs=(0,kk,0.28) , quat=(1,0,0,0)) # on the "table basse"
r.moveObject('beer',beer_pos)
r.addObject('stuff','stuff_base')
stuff_pos= Configuration (trs=(kk-0.2,-k-0.4,1.7) , quat=(0.5,0,0,0.5)) # on the "table basse"
r.moveObject('stuff',stuff_pos)
r(q0)

