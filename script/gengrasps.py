configs = list (); grasps = list (); success = 0

for i in range (100):
    res = p.client.manipulation.problem.applyConstraintsWithOffset ([id["pregrasp"]], qinit, robot.shootRandomConfig ())
    if res[0]:
        success = success + 1
        configs.append (res[1])

for c in configs:
    res = p.client.manipulation.problem.applyConstraintsWithOffset ([id["grasp"]], c, robot.shootRandomConfig ())       
    if res [0]:
        grasps.append ([c,res[1]])
