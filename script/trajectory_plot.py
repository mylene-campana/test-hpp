#/usr/bin/env python
# Author : Mylene Campana
# Script to plot some graphs after launching a corbaserver and solving 
# the problem.

import matplotlib.pyplot as plt
import numpy as np

dt = 0.1 # global drawing step size

def plannarPlot (cl):
    t_vec = np.arange(0., cl.problem.pathLength(0), dt)

    Tvec = t_vec [::]
    plt.subplot(221)
    circle_obst=plt.Circle((0,0),.4,color='r')
    plt.gcf().gca().add_artist(circle_obst)
    i = 0
    for n in cl.problem.nodes() :
        plt.plot(n[1], n[0], 'ro')
        if i>1: # avoid 2 first nodes (init and goal)
            plt.text(n[1]+.01, n[0], r'q_rand %i' %(i), fontsize=7)
        i=i+1

    for t in Tvec:
        plt.plot([cl.problem.configAtDistance(0, t)[1], \
                     cl.problem.configAtDistance(0, t+dt)[1]], \
                     [cl.problem.configAtDistance(0, t)[0], \
                     cl.problem.configAtDistance(0, t+dt)[0]], 'b')

    plt.axis([-3, 3, -3, 3]) #Ymin Ymax Xmin Xmax
    plt.xlabel('y')
    plt.ylabel('x')
    plt.title('trajectory')
    plt.grid()
    plt.gca().invert_xaxis()
    plt.text(.5, -.5, r'obstacle')
    plt.text(cl.problem.nodes()[0][1]+.2, cl.problem.nodes()[0][0]+.1, r'q_init')
    plt.text(cl.problem.nodes()[1][1]+.2, cl.problem.nodes()[1][0]+.1, r'q_end')

    plt.subplot(222)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(0, t)[0], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtDistance(0, t)[0], \
                                 cl.problem.configAtDistance(0, t+dt)[0]], 'k-')

    plt.axis([min(Tvec),max(Tvec),-3,3])
    plt.xlabel('t')
    plt.ylabel('x')
    plt.grid()

    plt.subplot(223)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(0, t)[1], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtDistance(0, t)[1], \
                                 cl.problem.configAtDistance(0, t+dt)[1]], 'k-')

    plt.axis([min(Tvec),max(Tvec),-3,3])
    plt.xlabel('t')
    plt.ylabel('y')
    plt.grid()

    plt.subplot(224)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(0, t)[2], 'ro') #theta
        cl.robot.setCurrentConfig(cl.problem.configAtDistance(0,t))
        plt.plot(t, cl.robot.distancesToCollision()[0], '.')

    plt.axis([min(Tvec),max(Tvec),-0.1,3.2])
    plt.xlabel('t')
    plt.ylabel('distance_obst')
    plt.grid()

    plt.show()
    #fig.savefig('plannarPlot.png')

# --------------------------------------------------------------------#

# Draw specified DoF(t) of the nPath (0 or 1)
def dofPlot (cl, nPath, n1, n2, n3, n4):
    t_vec = np.arange(0., cl.problem.pathLength(nPath), dt) # all
    t_ssvec = np.arange(4.2, 6., dt) # subvector

    Tvec = t_vec [::]
    plt.subplot(221)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(nPath, t)[n1], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtDistance(nPath, t)[n1], \
                                 cl.problem.configAtDistance(nPath, t+dt)[n1]], 'k-')

    plt.axis([min(Tvec),max(Tvec),-2,2])
    plt.xlabel('t')
    plt.ylabel('theta' + str(n1))
    plt.grid()

    plt.subplot(222)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(0, t)[42], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtDistance(nPath, t)[n2], \
                                 cl.problem.configAtDistance(nPath, t+dt)[n2]], 'k-')

    plt.axis([min(Tvec),max(Tvec),-2,2])
    plt.xlabel('t')
    plt.ylabel('theta' + str(n2))
    plt.grid()

    plt.subplot(223)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(0, t)[43], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtDistance(nPath, t)[n3], \
                                 cl.problem.configAtDistance(nPath, t+dt)[n3]], 'k-')

    plt.axis([min(Tvec),max(Tvec),-2,2])
    plt.xlabel('t')
    plt.ylabel('theta' + str(n3))
    plt.grid()

    plt.subplot(224)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(0, t)[45], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtDistance(nPath, t)[n4], \
                                 cl.problem.configAtDistance(nPath, t+dt)[n4]], 'k-')

    plt.axis([min(Tvec),max(Tvec),-2,2])
    plt.xlabel('t')
    plt.ylabel('theta' + str(n4))
    plt.grid()

    plt.show()
    #fig.savefig('dofPlot.png')


# --------------------------------------------------------------------#

# Draw gradAtt(t) and gradRep(t) for each x y dimension
def gradientPlot (gradAtt, gradRep):
    t_grad = np.arange(0., len(zip(*gradAtt))-1)

    plt.subplot(211)
    for t in t_grad:
        plt.plot([t, t+1], [gradAtt[0][t], gradAtt[0][t+1]],'g', label='gradAtt')
        plt.plot([t, t+1], [gradRep[0][t], gradRep[0][t+1]],'r', label='gradRep')

    plt.axis([min(t_grad),max(t_grad),min(min(gradRep[0]),min(gradAtt[0]))-5, max(max(gradRep[0]),max(gradAtt[0]))+5])
    plt.xlabel('t')
    plt.ylabel('Gradients (x) in first loop')
    plt.grid()

    plt.subplot(212)
    for t in t_grad:
        plt.plot([t, t+1], [gradAtt[1][t], gradAtt[1][t+1]],'g', label='gradAtt')
        plt.plot([t, t+1], [gradRep[1][t], gradRep[1][t+1]],'r', label='gradRep')

    plt.axis([min(t_grad),max(t_grad),min(min(gradRep[1]),min(gradAtt[1]))-5, max(max(gradRep[1]),max(gradAtt[1]))+5])
    plt.xlabel('t')
    plt.ylabel('Gradients (y) in first loop')
    plt.grid()

    plt.show()
    #fig.savefig('gradientPlot.png')


# --------------------------------------------------------------------#

# Draw only one graph with trajectory, RM nodes and gradient arrows
def gradArrowsPlot(cl, q_list, grad_list):
    
    # Trajectory, obstacle, RM nodes stuff :
    t_vec = np.arange(0., cl.problem.pathLength(0), dt)
    Tvec = t_vec [::]
    circle_obst=plt.Circle((0,0),.4,color='r')
    plt.gcf().gca().add_artist(circle_obst)
    i = 0
    for n in cl.problem.nodes() :
        plt.plot(n[1], n[0], 'ro')
        if i>1: # avoid 2 first nodes (init and goal)
            plt.text(n[1]+.01, n[0], r'q_rand %i' %(i), fontsize=7)
        i=i+1

    for t in Tvec:
        plt.plot([cl.problem.configAtDistance(0, t)[1], \
                     cl.problem.configAtDistance(0, t+dt)[1]], \
                     [cl.problem.configAtDistance(0, t)[0], \
                     cl.problem.configAtDistance(0, t+dt)[0]], 'b')

    plt.axis([-5, 5, -5, 5])
    plt.xlabel('y')
    plt.ylabel('x')
    plt.title('trajectory')
    plt.grid()
    plt.gca().invert_xaxis()
    plt.text(.5, -.5, r'obstacle')
    plt.text(cl.problem.nodes()[0][1]+.2, cl.problem.nodes()[0][0]+.1, r'q_init')
    plt.text(cl.problem.nodes()[1][1]+.2, cl.problem.nodes()[1][0]+.1, r'q_end')
    
    # Gradients Arrows stuff :
    # format : q_list[x, y, theta][t]
    # format : grad_list[x, y, theta][t]
    i=0
    for q in zip(*q_list) :
        print q
        x = q[0]# verifier len(q)=3
        y = q[1]
        dx = grad_list[0][i]*2
        dy = grad_list[1][i]*2
        plt.arrow(y, x, dy, dx, shape='full', lw=0.8, length_includes_head=True, \
         head_width=.015, head_length=0.03)
        i=i+1

    plt.show()
    #fig.savefig('gradientsArrowsPlot.png')

