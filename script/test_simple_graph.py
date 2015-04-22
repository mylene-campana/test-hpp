# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.hrp2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui

class ScrewGun (object):
  rootJointType = 'freeflyer'
  packageName = 'airbus_environment'
  meshPackageName = 'airbus_environment'
  urdfName = 'screw_gun'
  urdfSuffix = ""
  srdfSuffix = ""

Robot.urdfSuffix = '_capsule_mesh'
Robot.srdfSuffix = '_manipulation'

# Load HRP2 and a screwgun {{{3
robot = Robot ('hrp2-screw', 'hrp2')
ps = ProblemSolver (robot)
ps.selectPathPlanner ("M-RRT")
vf = ViewerFactory (ps)
vf.loadObjectModel (ScrewGun, 'screw_gun')
for d in ["hrp2", "screw_gun"]:
  robot.setJointBounds (d+"/base_joint_xyz", [-4,4,-4,4,-4,4])

ps.selectPathProjector ("Progressive", 0.2)
ps.setErrorThreshold (1e-3)
ps.setMaxIterations (40)
# 3}}}

# Define configurations {{{3
half_sitting = q = robot.getInitialConfig ()
q_init = half_sitting [::]
# Set initial position of screw-driver
# q_init [-7:] = [2, 1, 0.65, 0.7071067811865476, 0, -0.7071067811865475, 0]
q_init [-7:] = [2, 1, 0.65, 0.7071067811865476, 0.5, -0.5, 0]
# Open left hand
ilh = robot.rankInConfiguration ['hrp2/LARM_JOINT6']
q_init [ilh:ilh+6] = [0.75, -0.75, 0.75, -0.75, 0.75, -0.75]

q_goal = q_init [::]
q_goal [-7:] = [2, -1, 0.65, 0.7071067811865476, 0, -0.7071067811865475, 0]

# 3}}}

# Generate constraints {{{3
graph = ConstraintGraph (robot, 'graph')


jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['hrp2'] = list ()
jointNames['allButHRP2LeftArm'] = list ()
for n in jointNames['all']:
  if n.startswith ("hrp2"):
    jointNames['hrp2'].append (n)
  if not n.startswith ("hrp2/LARM"):
    jointNames['allButHRP2LeftArm'].append (n)

ps.addPassiveDofs ('hrp2', jointNames ['hrp2'])
graph.createGrasp ('l_grasp', 'hrp2/leftHand', 'screw_gun/handle2', 'hrp2')
graph.createPreGrasp ('l_pregrasp', 'hrp2/leftHand', 'screw_gun/handle2')

lockscrewgun = ps.lockFreeFlyerJoint ('screw_gun/base_joint', 'screwgun_lock',
    compType = 'Equality')

locklhand = ['larm_6','lhand_0','lhand_1','lhand_2','lhand_3','lhand_4']
ps.createLockedJoint ('larm_6' , 'hrp2/LARM_JOINT6' , [q_init[ilh],])
ps.createLockedJoint \
    ('lhand_0', 'hrp2/LHAND_JOINT0', [q_init[ilh + 1],])
ps.createLockedJoint \
    ('lhand_1', 'hrp2/LHAND_JOINT1', [q_init[ilh + 2],])
ps.createLockedJoint \
    ('lhand_2', 'hrp2/LHAND_JOINT2', [q_init[ilh + 3],])
ps.createLockedJoint \
    ('lhand_3', 'hrp2/LHAND_JOINT3', [q_init[ilh + 4],])
ps.createLockedJoint \
    ('lhand_4', 'hrp2/LHAND_JOINT4', [q_init[ilh + 5],])

ps.addPartialCom ('hrp2', ['hrp2/base_joint_xyz'])
ps.createStaticStabilityConstraints ("balance-hrp2", q_init, 'hrp2')
# 3}}}

# Create the graph of constraints {{{3
graph.createNode (["screwgun", "free"])

graph.createWaypointEdge ('screwgun', 'free', 'ungrasp', nb=1, weight=1)

graph.createWaypointEdge ('free', 'screwgun', 'grasp', nb=1, weight=5)

graph.createEdge ('free', 'free', 'move_free', 5)

graph.createEdge ('screwgun', 'screwgun', 'keep_grasp', 10)
graph.createLevelSetEdge ('screwgun', 'screwgun', 'keep_grasp_ls', 5)

graph.setConstraints (node='screwgun', grasp='l_grasp')
graph.setConstraints (edge='move_free', lockDof = lockscrewgun)
graph.setConstraints (edge='ungrasp_e0', lockDof = lockscrewgun)
graph.setConstraints (node='ungrasp_n0', pregrasp = 'l_pregrasp')
graph.setConstraints (edge='ungrasp_e1', lockDof = lockscrewgun)
graph.setConstraints (edge='grasp_e0', lockDof = lockscrewgun)
graph.setConstraints (node='grasp_n0', pregrasp = 'l_pregrasp')
graph.setConstraints (edge='grasp_e1', lockDof = lockscrewgun)
graph.setLevelSetConstraints ('keep_grasp_ls', lockDof = lockscrewgun)
graph.setConstraints (graph=True, lockDof = locklhand, numConstraints=ps.balanceConstraints ())
# 3}}}

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

r = vf.createRealClient()
pp = PathPlayer (robot.client.basic, r)
