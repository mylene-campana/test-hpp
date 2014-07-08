#/usr/bin/env python
# Author : Mylene Campana
# Script to plot some graphs after launching a corbaserver and solving 
# the problem.

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

dt = 0.1 # global drawing step size

def plannarPlot (cl):
    t_vec = np.arange(0., cl.problem.pathLength(0), dt)

    Tvec = t_vec [::]
    plt.subplot(221)
    circle_obst=plt.Circle((0,0),.5,color='r')
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
    plt.figure().savefig('plannarPlot.png')

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
    circle_obst=plt.Circle((0,0),.5,color='r')
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
                     cl.problem.configAtDistance(0, t+dt)[0]], 'b', \
                     label="trajectory" if t==0. else "")

    plt.axis([-3, 3, -3, 3])
    plt.xlabel('y'); plt.ylabel('x')
    plt.title('Trajectory and gradients'); plt.text(0, 0, r'obstacle')
    plt.grid(); plt.legend(); plt.gca().invert_xaxis()
    plt.text(cl.problem.nodes()[0][1]+.2, cl.problem.nodes()[0][0]+.15, r'q_init')
    plt.text(cl.problem.nodes()[1][1]+.2, cl.problem.nodes()[1][0]+.15, r'q_end')
    
    # Gradients Arrows stuff :
    # formats : q_list[x, y, theta][t] and grad_list[x, y, theta][t]
    i=0; thres1=0.8; thres2=1.; thres3=1.3
    boolPlot1=True; boolPlot2=True; boolPlot3=True; boolPlot4=True
    for q in zip(*q_list) :
        #print q
        x = q[0]
        y = q[1]
        dist = math.sqrt(x**2+y**2)
        
        if (dist <= thres1):
            dx = grad_list[0][i]*0.01
            dy = grad_list[1][i]*0.01
            arrow1=plt.arrow(y, x, dy, dx, lw=0.8,head_width=.015, head_length=0.025, fc='y', ec='y', label="arrow1" if boolPlot1 else "")
            boolPlot1=False
        
        elif (dist > thres1 and dist <= thres2):
            dx = grad_list[0][i]*0.07
            dy = grad_list[1][i]*0.07
            arrow2=plt.arrow(y, x, dy, dx, lw=0.8,head_width=.015, head_length=0.025, fc='0.9', ec='0.9', label="arrow2" if boolPlot2 else "")
            boolPlot2=False
        
        elif (dist > thres2 and dist <= thres3):
            dx = grad_list[0][i]*0.8
            dy = grad_list[1][i]*0.8
            arrow3=plt.arrow(y, x, dy, dx, lw=0.8,head_width=.015, head_length=0.025, fc='0.7', ec='0.7', label="arrow3" if boolPlot3 else "")
            boolPlot3=False
        
        elif (dist > thres3):
            dx = grad_list[0][i]*1.5
            dy = grad_list[1][i]*1.5       
            arrow4=plt.arrow(y, x, dy, dx, lw=0.8,head_width=.015, head_length=0.025, fc='0.5', ec='0.5', label="arrow4" if boolPlot4 else "")
            boolPlot4=False
        i=i+1
    
    plt.legend([arrow1, arrow2, arrow3, arrow4], ['coef=0.01, thres=0.8', 'coef 0.07, thres=1', 'coef 0.8, thres=1.3', 'coef 1.5'])
    plt.show()
    #fig.savefig('gradientsArrowsPlot.png')

# --------------------------------------------------------------------#

# 3D plot of translations :
def xyzPlot (cl):
    t_vec = np.arange(0., cl.problem.pathLength(0), dt)
    ax = plt.figure().gca(projection='3d')

    Tvec = t_vec [::]
    plt.subplot(221)
    circle_obst=plt.Circle((0,0),.5,color='r')
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
    ax.plot(x, y, z, label='parametric curve') # 3D
    # ax.legend() # 3D

    plt.show()
    fig.savefig('plannarPlot.png')  # plt.figure().savefig('plannarPlot.png')
