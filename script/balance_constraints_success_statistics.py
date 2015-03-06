##
## hpp-wholebody-step-server should be launched
##

from hpp.corbaserver import Client, ProblemSolver
from hpp.corbaserver.wholebody_step import Client as WSClient
from hpp.corbaserver.wholebody_step import Problem
from hpp.corbaserver.hrp2 import Robot

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix = '_capsule'

robot = Robot ('hrp2')
p = ProblemSolver (robot)
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
