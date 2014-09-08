from hpp.corbaserver.manipulation.hrp2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver
from scene_publisher3b import ScenePublisher as MultiRobotPub

#Robot.urdfSuffix = '_capsule_mesh_hands_fixed'
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
robot.client.basic.problem.setMaxIterations (30)

r = MultiRobotPub (robot)

half_sitting = robot.getCurrentConfig ();
q1=[0.2561522768052704, 0.009968182216397095, -0.0013343337623661373, 0.9822200021750263, -0.01324065709811156, 0.06488994582367685, 0.1756640181081545, 0.3518980306918394, 0.5587732909530961, -0.00014707196704839167, 0.003348199890377334, 0.2851538622381365, 0.019515985023849783, 0.47568241804625144, 0.034325840512149174, 0.48238375657832383, 0.5252200250587239, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.08183778724042397, -0.002329068502171773, 0.0013672024871911833, -0.03184796508559784, 0.00013468971902929027, -0.006463660186047965, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.0014454298679420635, 0.008692815372600783, -0.17944952816665183, -0.0349066, 0.08186718484328222, -0.0052176502711966674, -0.0014458867527617663, 0.008689383322591662, -0.1789250482962942, -0.0349066, 0.0813567181568653, -0.00521418794593098, -1.0, -0.009968182216397, 0.3399077826256168, 0.8920410834920809, 0.03081996727219969, -0.37686669705702674, -0.2475567159843218]
q2=[0.2561522768052704, 0.009968182216397094, -0.0013343337623677983, 0.9822200021750275, -0.013240657098102699, 0.06488994582365894, 0.17566401810815518, 0.3518980306918404, 0.5587732909530987, -0.00014707196704828986, 0.003348199890377807, 0.2851538622381267, 0.019515985023862044, 0.4756824180462511, 0.03432584051214365, 0.48238375657832383, 0.5252200250587228, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.08183778724043055, -0.0023290685021591065, 0.0013672024871913863, -0.03184796508560018, 0.00013468971902932646, -0.006463660186048368, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, -0.001445429867963691, 0.008692815372566085, -0.17944952816666074, -0.0349066, 0.08186718484332876, -0.00521765027117088, -0.0014458867527830789, 0.008689383322556907, -0.17892504829630115, -0.0349066, 0.08135671815690743, -0.005214187945905259, 1.0, -0.009968182216397, 0.3399077826256168, 0.8920410834920809, 0.03081996727219969, -0.37686669705702674, -0.2475567159843218]
q3=[0.2561522768052704, 0.009968182216397017, -0.0013072533849402136, 0.9822384333854748, -0.01312068217341367, 0.06461437559545073, 0.17567154051286343, 0.35191845881842604, 0.5584191613132159, -0.00014417077050741273, 0.003264525687460523, 0.2869014360284614, 0.01968512963006628, 0.47563871278773184, 0.0349066, 0.4823374727974044, 0.5253128675582199, 0.00028141477557467405, -9.224156430856334e-05, 9.57339438119022e-05, 0.00043012512285501805, 9.464640215865539e-05, -0.00027454688131241776, -0.08070554282100409, -0.0022150673918241194, 0.0013507126487957277, -0.031445080182081316, 0.00013303660804169588, -0.0063951082465386055, 0.00018071522532470678, -7.008398345853373e-05, 0.0001432918466161822, 5.0663492959904484e-05, 5.932458587315774e-05, -0.0001501990824631989, -0.0016207277515218107, 0.008331445043308924, -0.1782202190053313, -0.0349066, 0.08120820667180957, -0.004973436857255375, -0.0016211611493169267, 0.008328178972275535, -0.17768239866563243, -0.03490564559358417, 0.08066263806839956, -0.00497014215842449, -0.2561522768052704, -0.009968182216396924, 0.33990778262561817, 0.8920410834920809, 0.03081996727219969, -0.37686669705702674, -0.2475567159843218]

p = ProblemSolver (robot)
p.createGrasp ('left-hand-grasp', 'hrp2/leftHand', 'screw_gun/handle2')
p.createLockedDofConstraint ('screwgun_lock_x' , 'screw_gun/base_joint_x'  , 0, 0, 0)
p.createLockedDofConstraint ('screwgun_lock_y' , 'screw_gun/base_joint_y'  , 0, 0, 0)
p.createLockedDofConstraint ('screwgun_lock_z' , 'screw_gun/base_joint_z'  , 0, 0, 0)
p.createLockedDofConstraint ('screwgun_lock_rx', 'screw_gun/base_joint_SO3', 0, 1, 0)
p.createLockedDofConstraint ('screwgun_lock_ry', 'screw_gun/base_joint_SO3', 0, 2, 1)
p.createLockedDofConstraint ('screwgun_lock_rz', 'screw_gun/base_joint_SO3', 0, 3, 2)
p.isLockedDofParametric ('screwgun_lock_x' ,True)
p.isLockedDofParametric ('screwgun_lock_y' ,True)
p.isLockedDofParametric ('screwgun_lock_z' ,True)
p.isLockedDofParametric ('screwgun_lock_rx',True)
p.isLockedDofParametric ('screwgun_lock_ry',True)
p.isLockedDofParametric ('screwgun_lock_rz',True)
lockscrewgun = ['screwgun_lock_x', 'screwgun_lock_y', 'screwgun_lock_z', 'screwgun_lock_rx', 'screwgun_lock_ry', 'screwgun_lock_rz']

p.createStaticStabilityConstraints ("balance", q1)

graph = robot.client.manipulation.graph
id = dict()
id["graph"   ] = graph.createGraph ('hrp2-screwgun')
id["subgraph"] = graph.createSubGraph ('lefthand')
id["screwgun"] = graph.createNode (id["subgraph"], 'screwgun')
id["free"    ] = graph.createNode (id["subgraph"], 'free')
id["ungrasp"] = graph.createEdge (id["screwgun"], id["free"], "ungrasp", 1)
id[  "grasp"] = graph.createEdge (id["free"], id["screwgun"],   "grasp", 4)
id["move_free" ] = graph.createEdge (id["free"    ], id["free"    ], "move_free", 1)
id["keep_grasp"] = graph.createEdge (id["screwgun"], id["screwgun"], "keep_grasp", 4)

graph.setNumericalConstraints (id["screwgun"], ['left-hand-grasp'])
graph.setLockedDofConstraints (id["move_free"], lockscrewgun)
graph.setLockedDofConstraints (id["grasp"], lockscrewgun)
graph.setLockedDofConstraints (id["ungrasp"], lockscrewgun)
graph.setNumericalConstraints (id["graph"], p.balanceConstraints ())

manip = robot.client.manipulation

#res = manip.problem.applyConstraints ([id["free"]], robot.shootRandomConfig())
#if not res[0]:
  #raise StandardError ("Could not project qinit")
#qinit = res[1]

#res = manip.problem.applyConstraints ([id["free"]], robot.shootRandomConfig())
#if not res[0]:
  #raise StandardError ("Could not project qgoal")
#qgoal = res[1]

#p.setInitialConfig (qinit)
#p.addGoalConfig (qgoal)

p.setInitialConfig (q1)
p.addGoalConfig (q2)

# This projector tends to fail with probability 0.6 (with random configuration)
manip.graph.statOnConstraint ([id["grasp"]])
