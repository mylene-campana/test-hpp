##
## hpp-wholebody-step-server should be launched
##

from hpp.corbaserver import Client, ProblemSolver
from hpp.corbaserver.wholebody_step import Client as WSClient
from hpp.corbaserver.wholebody_step import Problem
from hpp.corbaserver.hrp2 import Robot
from hpp.gepetto import ViewerFactory


Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix = '_capsule'

robot = Robot ('hrp2')
p = ProblemSolver (robot)
vf = ViewerFactory (p)
wcl = WSClient ()
q0 = robot.getInitialConfig ()

constraintName = "balance"
wcl.problem.addStaticStabilityConstraints (constraintName, q0, robot.leftAnkle,
                                           robot.rightAnkle, "",
                                           Problem.SLIDING)
balanceConstraints = [constraintName + "/relative-com",
                      constraintName + "/relative-orientation",
                      constraintName + "/relative-position",
                      constraintName + "/orientation-left-foot",
                      constraintName + "/position-left-foot"]
c2name = "stability"
wcl.problem.addStabilityConstraints (c2name, q0, robot.leftAnkle,
                                     robot.rightAnkle, "", Problem.ALIGNED_COM)
b2C = [ c2name + "/com-between-feet",
        c2name + "/orientation-right",
        c2name + "/orientation-left",
        c2name + "/position-right",
        c2name + "/position-left"]

robot.setJointBounds ("base_joint_xyz", [-4,4,-4,4,-4,4])

def testConstraint (constraints, nbIter = 100):
  success = 0
  p.resetConstraints ()
  p.setNumericalConstraints ('test', constraints)
  for i in range (nbIter):
    res = p.applyConstraints (robot.shootRandomConfig ())
    if res[0]:
      success = success + 1
  print "Constraint ", constraints, " succeeds ", success * 100 / nbIter, " %"

for constraint in balanceConstraints:
  testConstraint ([constraint])

testConstraint (balanceConstraints)

for constraint in b2C:
  testConstraint ([constraint])

testConstraint (b2C)
