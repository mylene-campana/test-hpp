from hpp.corbaserver.manipulation.hrp2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from scene_publisher3b import ScenePublisher as MultiRobotPub

Robot.urdfSuffix = '_capsule_mesh'
Robot.srdfSuffix = '_mathieu'

robot = Robot ('hrp2')
robot.client.basic.problem.selectPathPlanner ("M-RRT")
robot.loadObjectModel ('screw_gun', 'freeflyer', 'airbus_environment', 'screw_gun_nomass', '', '')
robot.buildCompositeRobot ('hrp2-screw', ['hrp2', 'screw_gun'])
for d in ["hrp2", "screw_gun"]:
  for a in ["x","y","z"]:
    robot.setJointBounds (d+"/base_joint_"+a, [-4,4])
robot.client.basic.problem.resetRoadmap ()
robot.client.basic.problem.selectPathOptimizer ('None')
robot.client.basic.problem.setErrorThreshold (1e-3)
robot.client.basic.problem.setMaxIterations (40)

r = MultiRobotPub (robot)

half_sitting = robot.getCurrentConfig ();
q1=[0.2561522768052704, 0.009968182216397095, -0.0013343337623661373, 0.9822200021750263, -0.01324065709811156, 0.06488994582367685, 0.1756640181081545, 0.3518980306918394, 0.5587732909530961, -0.00014707196704839167, 0.003348199890377334, 0.2851538622381365, 0.019515985023849783, 0.47568241804625144, 0.034325840512149174, 0.48238375657832383, 0.5252200250587239, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.08183778724042397, -0.002329068502171773, 0.0013672024871911833, -0.03184796508559784, 0.00013468971902929027, -0.006463660186047965, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.0014454298679420635, 0.008692815372600783, -0.17944952816665183, -0.0349066, 0.08186718484328222, -0.0052176502711966674, -0.0014458867527617663, 0.008689383322591662, -0.1789250482962942, -0.0349066, 0.0813567181568653, -0.00521418794593098, -1.0, -0.009968182216397, 0.3399077826256168, 0.8920410834920809, 0.03081996727219969, -0.37686669705702674, -0.2475567159843218]
q2=[0.2561522768052704, 0.009968182216397094, -0.0013343337623677983, 0.9822200021750275, -0.013240657098102699, 0.06488994582365894, 0.17566401810815518, 0.3518980306918404, 0.5587732909530987, -0.00014707196704828986, 0.003348199890377807, 0.2851538622381267, 0.019515985023862044, 0.4756824180462511, 0.03432584051214365, 0.48238375657832383, 0.5252200250587228, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.08183778724043055, -0.0023290685021591065, 0.0013672024871913863, -0.03184796508560018, 0.00013468971902932646, -0.006463660186048368, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.001445429867963691, 0.008692815372566085, -0.17944952816666074, -0.0349066, 0.08186718484332876, -0.00521765027117088, -0.0014458867527830789, 0.008689383322556907, -0.17892504829630115, -0.0349066, 0.08135671815690743, -0.005214187945905259, 1.0, -0.009968182216397, 0.3399077826256168, 0.8920410834920809, 0.03081996727219969, -0.37686669705702674, -0.2475567159843218]
q3=[0.2561522768052704, 0.009968182216397017, -0.0013072533849402136, 0.9822384333854748, -0.01312068217341367, 0.06461437559545073, 0.17567154051286343, 0.35191845881842604, 0.5584191613132159, -0.00014417077050741273, 0.003264525687460523, 0.2869014360284614, 0.01968512963006628, 0.47563871278773184, 0.0349066, 0.4823374727974044, 0.5253128675582199, 0.00028141477557467405, -9.224156430856334e-05, 9.57339438119022e-05, 0.00043012512285501805, 9.464640215865539e-05, -0.00027454688131241776, -0.08070554282100409, -0.0022150673918241194, 0.0013507126487957277, -0.031445080182081316, 0.00013303660804169588, -0.0063951082465386055, 0.00018071522532470678, -7.008398345853373e-05, 0.0001432918466161822, 5.0663492959904484e-05, 5.932458587315774e-05, -0.0001501990824631989, -0.0016207277515218107, 0.008331445043308924, -0.1782202190053313, -0.0349066, 0.08120820667180957, -0.004973436857255375, -0.0016211611493169267, 0.008328178972275535, -0.17768239866563243, -0.03490564559358417, 0.08066263806839956, -0.00497014215842449, -0.2561522768052704, -0.009968182216396924, 0.33990778262561817, 0.8920410834920809, 0.03081996727219969, -0.37686669705702674, -0.2475567159843218]

qinit= [6.351445422051578e-15, -7.64857075802911e-16, -0.00010650933265040068, 0.9997086753828088, -0.001462864329132029, 0.024091994398753543, 1.4078154478767463e-05, -0.00040620371503693694, 0.06720468781884271, 0.09997150487212819, 0.10619128937732075, -0.030984458364104503, 0.06768151515542697, -9.21491333147205e-05, -0.007395092069129794, 4.108678412429508e-05, 0.09687147183171207, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.030157742792904667, -0.06148082407263112, 0.000178380167127975, -0.006607688410387017, 7.424531689237967e-05, 0.00026391550679217136, 0.0003776033936788175, 0.000250194383634261, -0.0005725514027501679, 6.818331370664385e-05, -0.00020049176175900826, 0.0004808871166639079, -0.0011302430393443214, 0.00792613666021656, -0.05431204743596144, -0.020779247681766293, 0.026905347698115048, -0.00494217897984072, -0.0011303560351055939, 0.007923626259361372, -0.05448958126663155, -0.019807309704424767, 0.02612488205235436, -0.004939666182290102, 2, -0.009968182216395699, 0.3399077826251962, 0.7071067811865476, 0, -0.7071067811865475, 0]
qgoal= [6.351445422051578e-15, -7.64857075802911e-16, -0.00010650933265040068, 0.9997086753828088, -0.001462864329132029, 0.024091994398753543, 1.4078154478767463e-05, -0.00040620371503693694, 0.06720468781884271, 0.09997150487212819, 0.10619128937732075, -0.030984458364104503, 0.06768151515542697, -9.21491333147205e-05, -0.007395092069129794, 4.108678412429508e-05, 0.09687147183171207, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.030157742792904667, -0.06148082407263112, 0.000178380167127975, -0.006607688410387017, 7.424531689237967e-05, 0.00026391550679217136, 0.0003776033936788175, 0.000250194383634261, -0.0005725514027501679, 6.818331370664385e-05, -0.00020049176175900826, 0.0004808871166639079, -0.0011302430393443214, 0.00792613666021656, -0.05431204743596144, -0.020779247681766293, 0.026905347698115048, -0.00494217897984072, -0.0011303560351055939, 0.007923626259361372, -0.05448958126663155, -0.019807309704424767, 0.02612488205235436, -0.004939666182290102, 1, -0.009968182216395699, 0.3399077826251962, 0.7071067811865476, 0, -0.7071067811865475, 0]

p = ProblemSolver (robot)
p.createGrasp ('left-hand-grasp', 'hrp2/leftHand', 'screw_gun/handle2')
p.createGrasp ('left-hand-grasp-passive', 'hrp2/leftHand', 'screw_gun/handle2')
p.createPreGrasp ('pregrasp', 'hrp2/leftHand', 'screw_gun/handle2')
rankcfg = 1; rankvel = 0; lockscrewgun = list ();
for axis in ['x','y','z']:
  p.createLockedDofConstraint ('screwgun_lock_'  + axis, 'screw_gun/base_joint_' + axis, 0, 0, 0)
  p.createLockedDofConstraint ('screwgun_lock_r' + axis, 'screw_gun/base_joint_SO3', 0, rankcfg, rankvel)
  p.isLockedDofParametric ('screwgun_lock_'  + axis ,True)
  p.isLockedDofParametric ('screwgun_lock_r' + axis ,True)
  lockscrewgun.append ('screwgun_lock_'  + axis)
  lockscrewgun.append ('screwgun_lock_r' + axis)
  rankcfg = rankcfg + 1
  rankvel = rankvel + 1

locklhand = ['larm_6','lhand_0','lhand_1','lhand_2','lhand_3','lhand_4']
p.createLockedDofConstraint ('larm_6' , 'hrp2/LARM_JOINT6' , q1[17], 0, 0)
p.createLockedDofConstraint ('lhand_0', 'hrp2/LHAND_JOINT0', q1[18], 0, 0)
p.createLockedDofConstraint ('lhand_1', 'hrp2/LHAND_JOINT1', q1[19], 0, 0)
p.createLockedDofConstraint ('lhand_2', 'hrp2/LHAND_JOINT2', q1[20], 0, 0)
p.createLockedDofConstraint ('lhand_3', 'hrp2/LHAND_JOINT3', q1[21], 0, 0)
p.createLockedDofConstraint ('lhand_4', 'hrp2/LHAND_JOINT4', q1[22], 0, 0)

jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['hrp2'] = list ()
jointNames['allButHRP2LeftArm'] = list ()
for n in jointNames['all']:
  if n.startswith ("hrp2"):
    jointNames['hrp2'].append (n)
  if not n.startswith ("hrp2/LARM"):
    jointNames['allButHRP2LeftArm'].append (n)
robot.client.basic.problem.setPassiveDofs ('left-hand-grasp-passive', jointNames['hrp2'])

jointNames ["bottomPart"] = \
  ['hrp2/base_joint_x',
   'hrp2/base_joint_y',
   'hrp2/base_joint_z',
   'hrp2/base_joint_SO3',
   'hrp2/LLEG_JOINT0',
   'hrp2/LLEG_JOINT1',
   'hrp2/LLEG_JOINT2',
   'hrp2/LLEG_JOINT3',
   'hrp2/LLEG_JOINT4',
   'hrp2/LLEG_JOINT5',
   'hrp2/RLEG_JOINT0',
   'hrp2/RLEG_JOINT1',
   'hrp2/RLEG_JOINT2',
   'hrp2/RLEG_JOINT3',
   'hrp2/RLEG_JOINT4',
   'hrp2/RLEG_JOINT5']

lockbottompart = list ()
for n in jointNames ["bottomPart"]:
  if   n is "hrp2/base_joint_SO3":
    lockbottompart.append ("lock"+n+'_x')
    p.createLockedDofConstraint (lockbottompart[-1], n, 0, 1 ,0) 
    p.isLockedDofParametric (lockbottompart[-1], True)

    lockbottompart.append ("lock"+n+'_y')
    p.createLockedDofConstraint (lockbottompart[-1], n, 0, 2 ,1) 
    p.isLockedDofParametric (lockbottompart[-1], True)

    lockbottompart.append ("lock"+n+'_z')
    p.createLockedDofConstraint (lockbottompart[-1], n, 0, 3 ,2) 
    p.isLockedDofParametric (lockbottompart[-1], True)
  else:
    lockbottompart.append ("lock"+n)
    p.createLockedDofConstraint (lockbottompart[-1], n, 0, 0, 0)
    p.isLockedDofParametric (lockbottompart[-1], True)

p.createStaticStabilityConstraints ("balance", q1)

graph = robot.client.manipulation.graph
id = dict()
id["graph"   ] = graph.createGraph ('hrp2-screwgun')
id["subgraph"] = graph.createSubGraph ('lefthand')
id["screwgun"] = graph.createNode (id["subgraph"], 'screwgun')
id["free"    ] = graph.createNode (id["subgraph"], 'free')

id["ungrasp"] = graph.createWaypointEdge (id["screwgun"], id["free"], "ungrasp", 1, False)
id["ungrasp_w"], id["ungrasp_w_node"] = graph.getWaypoint (id["ungrasp"])

id[  "grasp"] = graph.createWaypointEdge (id["free"], id["screwgun"],   "grasp", 5, True)
id["grasp_w"], id["grasp_w_node"] = graph.getWaypoint (id["grasp"])

id["move_free" ] = graph.createEdge (id["free"    ], id["free"    ], "move_free" , 1 , False)

id["keep_grasp"] = graph.createEdge (id["screwgun"], id["screwgun"], "keep_grasp", 5, False)
id["keep_grasp_ls"] = graph.createLevelSetEdge (id["screwgun"], id["screwgun"], "keep_grasp_ls", 5, False)

graph.setNumericalConstraints (id["screwgun"], ['left-hand-grasp'])
graph.setNumericalConstraintsForPath (id["screwgun"], ['left-hand-grasp-passive'])
graph.setLockedDofConstraints (id["move_free"], lockscrewgun)
graph.setLockedDofConstraints (id["ungrasp"], lockscrewgun)
graph.setNumericalConstraints (id["ungrasp_w_node"], ['pregrasp', 'pregrasp/ineq_0', 'pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["ungrasp_w"], lockscrewgun)
graph.setLockedDofConstraints (id["grasp"], lockscrewgun)
graph.setNumericalConstraints (id["grasp_w_node"], ['pregrasp', 'pregrasp/ineq_0', 'pregrasp/ineq_0.1'])
graph.setLockedDofConstraints (id["grasp_w"], lockscrewgun)
graph.setLevelSetConstraints  (id["keep_grasp_ls"], [], lockscrewgun)
graph.setNumericalConstraints (id["graph"], p.balanceConstraints ())
graph.setLockedDofConstraints (id["graph"], locklhand)

manip = robot.client.manipulation

p.setInitialConfig (qinit)
p.addGoalConfig (qgoal)

# This projector tends to fail with probability 0.6 (with random configuration)
#manip.graph.statOnConstraint (id["grasp"])
