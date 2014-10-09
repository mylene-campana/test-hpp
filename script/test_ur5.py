from hpp.corbaserver.manipulation.ur5 import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from scene_publisher3b import ScenePublisher as MultiRobotPub

Robot.packageName = "fiad_description"
Robot.urdfName = "ur5"
Robot.urdfSuffix = ''
Robot.srdfSuffix = '_ee'
withWaypoint = True
withLevelSetEgde = True

robot = Robot ('ur5')
robot.client.basic.problem.selectPathPlanner ("M-RRT")
robot.loadObjectModel ('box', 'freeflyer', 'fiad_description', 'box', '', '')
robot.buildCompositeRobot ('robot', ['ur5', 'box'])
for a in ["x","y","z"]:
  robot.setJointBounds ("box/base_joint_"+a, [-1,1])
robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.selectPathOptimizer ('None')
robot.client.basic.problem.setMaxIterations (40)

r = MultiRobotPub (robot)

half_sitting = robot.getCurrentConfig ();
qinit=[0,0,0,0,0,0,0, 0.5,0,1,0,0,0]
qgoal=[0,0,0,0,0,0,0,-0.5,0,1,0,0,0]

p = ProblemSolver (robot)
p.createGrasp ('grasp', 'ur5/ee', 'box/handle')
p.createGrasp ('grasp-passive', 'ur5/ee', 'box/handle')
if withWaypoint:
  p.createPreGrasp ('pregrasp', 'ur5/ee', 'box/handle')
p.createLockedDofConstraint ('box_lock_x' , 'box/base_joint_x'  , 0, 0, 0)
p.createLockedDofConstraint ('box_lock_y' , 'box/base_joint_y'  , 0, 0, 0)
p.createLockedDofConstraint ('box_lock_z' , 'box/base_joint_z'  , 0, 0, 0)
p.createLockedDofConstraint ('box_lock_rx', 'box/base_joint_SO3', 0, 1, 0)
p.createLockedDofConstraint ('box_lock_ry', 'box/base_joint_SO3', 0, 2, 1)
p.createLockedDofConstraint ('box_lock_rz', 'box/base_joint_SO3', 0, 3, 2)
p.isLockedDofParametric ('box_lock_x' ,True)
p.isLockedDofParametric ('box_lock_y' ,True)
p.isLockedDofParametric ('box_lock_z' ,True)
p.isLockedDofParametric ('box_lock_rx',True)
p.isLockedDofParametric ('box_lock_ry',True)
p.isLockedDofParametric ('box_lock_rz',True)
lockbox = ['box_lock_x', 'box_lock_y', 'box_lock_z', 'box_lock_rx', 'box_lock_ry', 'box_lock_rz']

jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['ur5'] = list ()
for n in jointNames['all']:
  if n.startswith ("ur5"):
    jointNames['ur5'].append (n)
robot.client.basic.problem.setPassiveDofs ('grasp-passive', jointNames['ur5'])

graph = robot.client.manipulation.graph
id = dict()
id["graph"   ] = graph.createGraph ('ur5-box')
id["subgraph"] = graph.createSubGraph ('ee')
id["box" ] = graph.createNode (id["subgraph"], 'box' )
id["free"] = graph.createNode (id["subgraph"], 'free')

id["ungrasp"] = graph.createEdge (id["box"],  id["free"],"ungrasp", 1, False)

id["move_free" ] = graph.createEdge (id["free"], id["free"], "move_free" , 1, False)
id["keep_grasp"] = graph.createEdge (id["box" ], id["box" ], "keep_grasp", 1, False)
if withLevelSetEgde:
  id["keep_grasp_ls"] = graph.createLevelSetEdge (id["box" ], id["box" ], "keep_grasp_ls", 1, False)

if withWaypoint:
  id[  "grasp"] = graph.createWaypointEdge (id["free"],  id["box"],  "grasp", 10, True)
  id["grasp_w"], id["grasp_w_node"] = graph.getWaypoint (id["grasp"])
else:
  id[  "grasp"] = graph.createEdge (id["free"],  id["box"],  "grasp", 10, True)

graph.setNumericalConstraints (id["box"], ['grasp'])
graph.setNumericalConstraintsForPath (id["box"], ['grasp-passive'])
graph.setLockedDofConstraints (id["move_free"], lockbox)
graph.setLockedDofConstraints (id["grasp"], lockbox)
graph.setLockedDofConstraints (id["ungrasp"], lockbox)
if withWaypoint:
  graph.setNumericalConstraints (id["grasp_w_node"], ['pregrasp', 'pregrasp/ineq_0', 'pregrasp/ineq_0.1'])
  graph.setLockedDofConstraints (id["grasp_w"], lockbox)
if withLevelSetEgde:
  graph.setLevelSetConstraints (id["keep_grasp_ls"], [], lockbox)

manip = robot.client.manipulation

p.setInitialConfig (qinit)
p.addGoalConfig (qgoal)

if not withLevelSetEgde:
  manip.graph.statOnConstraint (id["grasp"])
