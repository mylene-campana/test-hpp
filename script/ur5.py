# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.robot import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer
from math import pi as PI

# Load robot and object. {{{3

Robot.packageName = 'ur_description'
Robot.meshPackageName = 'ur_description'
Robot.urdfName = 'ur5'
Robot.urdfSuffix = '_robot'
Robot.srdfSuffix = ''

class Box (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""

robot = Robot ('ur5-box', 'ur5', rootJointType = 'anchor')
ps = ProblemSolver (robot)
fk = ViewerFactory (ps)

fk.loadObjectModel (Box, 'box')
fk.buildCompositeRobot (['ur5', 'box'])
robot.setJointBounds ("box/base_joint_xyz", [-2,+2,-2,+2,-2,2])
# 3}}}

# Define configurations. {{{3
q_init = robot.getCurrentConfig ()
rank = robot.rankInConfiguration ['ur5/shoulder_lift_joint']
q_init [rank] = 0
rank = robot.rankInConfiguration ['ur5/elbow_joint']
q_init [rank] = - PI / 2
rank = robot.rankInConfiguration ['box/base_joint_xyz']
q_init [rank:rank+3] = [1,1,1]
rank = robot.rankInConfiguration ['box/base_joint_SO3']
q_init [rank:rank+4] = [1,0,0,0]

q_goal = q_init [:]
rank = robot.rankInConfiguration ['ur5/shoulder_lift_joint']
q_goal [rank] = - PI
rank = robot.rankInConfiguration ['ur5/elbow_joint']
q_goal [rank] = PI / 2
# 3}}}

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.selectPathOptimizer ('None')
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (40)

# Create constraints. {{{3
lockbox = ps.lockFreeFlyerJoint ('box/base_joint', 'box_lock', values=(1, 1, 1, 1,0,0,0))

ps.createLockedJoint ('shoulder_pan', 'ur5/shoulder_pan_joint', [0])
ps.createLockedJoint ('wrist_1', 'ur5/wrist_1_joint', [0])
ps.createLockedJoint ('wrist_2', 'ur5/wrist_2_joint', [0])
ps.createLockedJoint ('wrist_3', 'ur5/wrist_3_joint', [0])
lockur5 = ['shoulder_pan','wrist_1', 'wrist_2', 'wrist_3'];

lockjoints = list ()
lockjoints.extend (lockbox)
lockjoints.extend (lockur5)

robot.setCurrentConfig (q_init)
tf = robot.getJointPosition ('ur5/wrist_1_joint')
#robot.client.basic.problem.createPositionConstraint \
#('pos', '', 'ur5/wrist_1_joint', (0, tf[1], tf[2]), (0,0,0), (False, True, True))
robot.client.basic.problem.createPositionConstraint \
('pos', '', 'ur5/wrist_1_joint', (0, 0, tf[2]), (0,0,0), (False, False, True))

# 3}}}

# Create the graph. {{{3
from hpp.corbaserver.manipulation import ConstraintGraph
cg = ConstraintGraph (robot, 'graph')

cg.createNode (['free'])

cg.setConstraints (node = 'free', numConstraints = ['pos'])

cg.setConstraints (graph = True, lockDof = lockjoints)

cg.createEdge ('free', 'free', 'move_free', 1)
# 3}}}

res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_init)
if not res[0]:
  raise Exception ('Init configuration could not be projected.')
q_init_proj = res [1]
res = ps.client.manipulation.problem.applyConstraints (cg.nodes['free'], q_goal)
if not res[0]:
  raise Exception ('Goal configuration could not be projected.')
q_goal_proj = res [1]

ps.setInitialConfig (q_init_proj)
ps.addGoalConfig (q_goal_proj)
