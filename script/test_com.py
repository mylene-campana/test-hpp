import numpy as np
from hpp.corbaserver import Client
from hrp2 import getHalfSitting

cl = Client ()
cl.robot.loadRobotModel ('hrp2_14', '_capsule', '_capsule', '')
jn = cl.robot.getJointNames ()
q0 = getHalfSitting (jn)
cl.robot.setCurrentConfig (q0)

com0 = np.matrix ([cl.robot.getCenterOfMass ()]).transpose ()
Jcom = np.matrix (cl.robot.getJacobianCenterOfMass ())

delta = 1e-6
for i in range (7, Jcom.shape [1]):
    q = q0 [::]; q [i+1] += delta
    cl.robot.setCurrentConfig (q)
    com = np.matrix ([cl.robot.getCenterOfMass ()]).transpose ()
    error = (com - com0)/delta - Jcom [:,i]
    print ("i=%i, error=%f"%(i, np.linalg.norm (error)))

