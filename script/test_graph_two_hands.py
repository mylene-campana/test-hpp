from hpp.corbaserver.manipulation.pr2 import Robot
from math import sqrt

robot = Robot ('pr2', rootJointType = "anchor")
robot.client.manipulation.robot.setRootJointPosition ('pr2', (-3.2, -4, 0, 1, 0, 0, 0))
robot.loadObjectModel ('box', 'freeflyer', 'hpp_tutorial', 'box', '', '')
robot.buildCompositeRobot ('pr2-box', ['pr2', 'box'])
robot.client.basic.obstacle.loadObstacleModel ("iai_maps", "kitchen_area", "")

robot.setJointBounds ("box/base_joint_xyz", [-3,-2,-5,-3,0.7,1])

from hpp_ros.manipulation import ScenePublisher
r = ScenePublisher (robot)

q_init = robot.getCurrentConfig ()
rank = robot.rankInConfiguration ['pr2/r_gripper_l_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/r_gripper_r_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_gripper_l_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/l_gripper_r_finger_joint']
q_init [rank] = 0.5
rank = robot.rankInConfiguration ['pr2/torso_lift_joint']
q_init [rank] = 0.2
q_goal = q_init [::]

rank = robot.rankInConfiguration ['box/base_joint_xyz']
q_init [rank:rank+3] = [-2.5, -3.6, 0.76]
rank = robot.rankInConfiguration ['box/base_joint_SO3']
c = sqrt (2) / 2
q_init [rank:rank+4] = [c, 0, c, 0]
r (q_init)

rank = robot.rankInConfiguration ['box/base_joint_xyz']
q_goal [rank:rank+3] = [-2.5, -4.4, 0.76]
rank = robot.rankInConfiguration ['box/base_joint_SO3']
q_goal [rank:rank+4] = [c, 0, -c, 0]
r (q_goal)
del c

q_inter = [-0.8017105239677402, -0.5977125025958312, -0.796440524800078, 0.6047168680102916, -0.4879605145323316, 0.8728657034488995, -0.988715265911429, 0.1498069522875767, -0.012487804646172175, -0.9999220243274567, -0.9479271854727237, -0.31848712853388694, 0.06700549180802896, -0.9977526066453368, 0.15793164217459785, 0.9874500475467277, 0.9804271799015071, -0.19688205837601827, 0.06981400674906149, 0.9975600254930236, 0.8026666074307995, -0.5964279649006498, -0.8558688410761539, -0.5171929300318802, 0.07633365848037467, 2.5514381844999448, 1.1951774265118278, -0.5864281075389233, 0.24807300661555917, 1.0730239901832896, -0.9931474461781542, 0.5380253563010143, -0.8429286541440898, 0.0, -0.9291311234626017, 0.36975039609596555, 0.5, 0.07192830903452277, 0.0516497980242827, 0.5, 0.2978673015357309, 0.011873305063635719, 0.2207828272342602, 0.9968680838221816, -1.1330407965502385, 0.1474961939381539, -0.9059397450606351, -0.9591666722669869, 0.8241613711518598, -0.5663550426199861, -2.094, 0.7924979452316735, 0.6098745828476339, 0.5, 0.35889873246983567, 0.10845342945887403, 0.5, 0.02916333341652683, 0.025597731231524482, 0.16145134862579935, -2.785939904956431, -4.563075760833335, 0.8958690128585236, -0.19634763254425533, -0.7205092487114027, 0.6650461296003519, 0.005260724207565836]

robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.selectPathOptimizer ('None')
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (40)

from hpp.corbaserver.manipulation import ProblemSolver
p = ProblemSolver (robot)

h = 0.05 / 2;
bds = [-3.1,-1.9,-5.1,-2.9]
l = 0.76 - h
pts = [[-h,-h,-h], [-h, h,-h], [-h,-h, h], [-h, h, h],
       [ h,-h,-h], [ h, h,-h], [ h,-h, h], [ h, h, h],
       [bds[0],bds[2], l],[bds[0],bds[3], l],[bds[1],bds[2], l],[bds[1],bds[3], l]]

oTri = [[0, 2, 1], [4, 5, 6]]
tTri = [[8, 9, 10], [9, 11, 10]]

robot.client.basic.problem.createStaticStabilityGravityConstraint ('box_placement', 'box/base_joint_SO3', pts, oTri, tTri)

p.createGrasp ('l_grasp', 'pr2/l_gripper', 'box/handle')
p.createGrasp ('l_grasp_passive', 'pr2/l_gripper', 'box/handle')

p.createGrasp ('r_grasp', 'pr2/r_gripper', 'box/handle2')
p.createGrasp ('r_grasp_passive', 'pr2/r_gripper', 'box/handle2')

p.createPreGrasp ('l_pregrasp', 'pr2/l_gripper', 'box/handle')
p.createPreGrasp ('r_pregrasp', 'pr2/r_gripper', 'box/handle2')

rankcfg = 0; rankvel = 0; lockbox = list ();
for axis in ['x','y','z']:
  p.createLockedDofConstraint ('box_lock_'  + axis, 'box/base_joint_xyz', 0, rankcfg, rankvel)
  p.createLockedDofConstraint ('box_lock_r' + axis, 'box/base_joint_SO3', 0, rankcfg + 1, rankvel)
  p.isLockedDofParametric ('box_lock_'  + axis ,True)
  p.isLockedDofParametric ('box_lock_r' + axis ,True)
  lockbox.append ('box_lock_'  + axis)
  lockbox.append ('box_lock_r' + axis)
  rankcfg = rankcfg + 1
  rankvel = rankvel + 1

locklhand = ['l_l_finger','l_r_finger'];
p.createLockedDofConstraint ('l_l_finger', 'pr2/l_gripper_l_finger_joint', 0.5, 0, 0)
p.createLockedDofConstraint ('l_r_finger', 'pr2/l_gripper_r_finger_joint', 0.5, 0, 0)

lockrhand = ['r_l_finger','r_r_finger'];
p.createLockedDofConstraint ('r_l_finger', 'pr2/r_gripper_l_finger_joint', 0.5, 0, 0)
p.createLockedDofConstraint ('r_r_finger', 'pr2/r_gripper_r_finger_joint', 0.5, 0, 0)

lockhands = ['l_l_finger','l_r_finger','r_l_finger','r_r_finger'];

jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['pr2'] = list ()
jointNames['allButPR2LeftArm'] = list ()
for n in jointNames['all']:
  if n.startswith ("pr2"):
    jointNames['pr2'].append (n)
  if not n.startswith ("pr2/l_gripper"):
    jointNames['allButPR2LeftArm'].append (n)
robot.client.basic.problem.setPassiveDofs ('l_grasp_passive', jointNames['pr2'])
robot.client.basic.problem.setPassiveDofs ('r_grasp_passive', jointNames['pr2'])

graph = robot.client.manipulation.graph
id = dict()
id["graph"   ] = graph.createGraph ('pr2-box')
id["subgraph"] = graph.createSubGraph ('grasps')

id["both" ] = graph.createNode (id["subgraph"], 'both' )
id["right"] = graph.createNode (id["subgraph"], 'right')
id["left" ] = graph.createNode (id["subgraph"], 'left' )
id["free" ] = graph.createNode (id["subgraph"], 'free' )

graph.setNumericalConstraints (id["free"], ['box_placement'])
# This is not need as the object is locked in node free.
#graph.setNumericalConstraintsForPath (id["free"], ['box_placement'])

### Right hand ###
graph.setNumericalConstraints (id["right"], ['r_grasp'])
graph.setNumericalConstraintsForPath (id["right"], ['r_grasp_passive'])

id["r_ungrasp"] = graph.createWaypointEdge (id["right"], id["free"], "r_ungrasp", 1, False)
id["r_ungrasp_w"], id["r_ungrasp_w_node"] = graph.getWaypoint (id["r_ungrasp"])

id["r_grasp"] = graph.createWaypointEdge (id["free"], id["right"], "r_grasp", 10, True)
id["r_grasp_w"], id["r_grasp_w_node"] = graph.getWaypoint (id["r_grasp"])

graph.setLockedDofConstraints (id["r_ungrasp"], lockbox)
graph.setNumericalConstraints (id["r_ungrasp_w_node"], ['r_pregrasp', 'r_pregrasp/ineq_0', 'r_pregrasp/ineq_0.1','box_placement'])
graph.setLockedDofConstraints (id["r_ungrasp_w"], lockbox)
graph.setLockedDofConstraints (id["r_grasp"], lockbox)
graph.setNumericalConstraints (id["r_grasp_w_node"], ['r_pregrasp', 'r_pregrasp/ineq_0', 'r_pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["r_grasp_w"], lockbox)

### Left hand ###
graph.setNumericalConstraints (id["left"], ['l_grasp'])
graph.setNumericalConstraintsForPath (id["left"], ['l_grasp_passive'])

id["l_ungrasp"] = graph.createWaypointEdge (id["left"], id["free"], "l_ungrasp", 1, False)
id["l_ungrasp_w"], id["l_ungrasp_w_node"] = graph.getWaypoint (id["l_ungrasp"])

id["l_grasp"  ] = graph.createWaypointEdge (id["free"], id["left"],   "l_grasp", 10, True)
id["l_grasp_w"], id["l_grasp_w_node"] = graph.getWaypoint (id["l_grasp"])

graph.setLockedDofConstraints (id["l_ungrasp"], lockbox)
graph.setNumericalConstraints (id["l_ungrasp_w_node"], ['l_pregrasp', 'l_pregrasp/ineq_0', 'l_pregrasp/ineq_0.1','box_placement'])
graph.setLockedDofConstraints (id["l_ungrasp_w"], lockbox)
graph.setLockedDofConstraints (id["l_grasp"], lockbox)
graph.setNumericalConstraints (id["l_grasp_w_node"], ['l_pregrasp', 'l_pregrasp/ineq_0', 'l_pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["l_grasp_w"], lockbox)

### Both hands ###
graph.setNumericalConstraints (id["both"], ['l_grasp', 'r_grasp'])
graph.setNumericalConstraintsForPath (id["both"], ['l_grasp', 'r_grasp'])

id["b_l_ungrasp"] = graph.createWaypointEdge (id["both"], id["right"], "b_l_ungrasp", 1, False)
id["b_l_ungrasp_w"], id["b_l_ungrasp_w_node"] = graph.getWaypoint (id["b_l_ungrasp"])

id["b_l_grasp"  ] = graph.createWaypointEdge (id["right"], id["both"],   "b_l_grasp", 10, True)
id["b_l_grasp_w"], id["b_l_grasp_w_node"] = graph.getWaypoint (id["b_l_grasp"])

graph.setLockedDofConstraints (id["b_l_ungrasp"], lockbox)
graph.setNumericalConstraints (id["b_l_ungrasp_w_node"], ['r_grasp', 'l_pregrasp', 'l_pregrasp/ineq_0', 'l_pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["b_l_ungrasp_w"], lockbox)
graph.setLockedDofConstraints (id["b_l_grasp"], lockbox)
graph.setNumericalConstraints (id["b_l_grasp_w_node"], ['r_grasp', 'l_pregrasp', 'l_pregrasp/ineq_0', 'l_pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["b_l_grasp_w"], lockbox)

id["b_r_ungrasp"] = graph.createWaypointEdge (id["both"], id["left"], "b_r_ungrasp", 1, False)
id["b_r_ungrasp_w"], id["b_r_ungrasp_w_node"] = graph.getWaypoint (id["b_r_ungrasp"])

id["b_r_grasp"  ] = graph.createWaypointEdge (id["left"], id["both"],   "b_r_grasp", 10, True)
id["b_r_grasp_w"], id["b_r_grasp_w_node"] = graph.getWaypoint (id["b_r_grasp"])

graph.setLockedDofConstraints (id["b_r_ungrasp"], lockbox)
graph.setNumericalConstraints (id["b_r_ungrasp_w_node"], ['l_grasp', 'r_pregrasp', 'r_pregrasp/ineq_0', 'r_pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["b_r_ungrasp_w"], lockbox)
graph.setLockedDofConstraints (id["b_r_grasp"], lockbox)
graph.setNumericalConstraints (id["b_r_grasp_w_node"], ['l_grasp', 'r_pregrasp', 'r_pregrasp/ineq_0', 'r_pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["b_r_grasp_w"], lockbox)

### Loops ###
id["move_free" ] = graph.createEdge (id["free"], id["free"], "move_free" , 1 , False)

id["l_keep_grasp"] = graph.createEdge (id["left"], id["left"], "l_keep_grasp", 5, False)
id["l_keep_grasp_ls"] = graph.createLevelSetEdge (id["left"], id["left"], "l_keep_grasp_ls", 10, False)

id["r_keep_grasp"] = graph.createEdge (id["right"], id["right"], "r_keep_grasp", 5, False)
id["r_keep_grasp_ls"] = graph.createLevelSetEdge (id["right"], id["right"], "r_keep_grasp_ls", 10, False)

graph.setLockedDofConstraints (id["move_free"], lockbox)
graph.setLevelSetConstraints (id["l_keep_grasp_ls"], [], lockbox)
graph.setLevelSetConstraints (id["r_keep_grasp_ls"], [], lockbox)

graph.setLockedDofConstraints (id["graph"], lockhands)

p.setInitialConfig (q_init)
p.addGoalConfig (q_goal)
