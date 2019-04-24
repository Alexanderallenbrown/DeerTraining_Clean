from numpy import *
from matplotlib.pyplot import *
import os
### CREATE ANIMATION
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
sys.path.append('../')

import cv2

#what is the collision threshold?
collision_thresh=1.0

#numbers to identify what sims we're looking at
setSpeed = 25.0
swerveDistance = 80.0
sightDistance = 80.0
car_x_offset = 0 #where are we on the map


#set up the parameters for the MPC
qle,qoe,qlv,qse,qa,predm = 10,200,1,1,.005,'CV'


#where are these runs going to be stored?
basedir = 'TrialData'


#what kind of driver is this?
controllertype = 'MPC_F_braking_KinCar'

#is this sam's deer, or the map deer?
deertype = 'Deer_Map'
deermap = 'Test2.kml'#MUST specify this. Will become important.

#what deer are we testing?
genome = '1100101100110111011110101'

basedir = 'TrialData'
outputdir=basedir+'/'+controllertype+'_'+str(int(qle))+'_'+str(int(qoe))+'_'+str(int(qlv))+'_'+str(int(qse))+'_'+str(int(qa))+'_'+str(predm)+'_sight_'+str(int(sightDistance))+'_swerve_'+str(int(swerveDistance))+'/'+deertype+'/'+deermap[0:-4]+'/'+'S'+str(int(car_x_offset))+'/'+'speed'+str(int(setSpeed))+'/'+genome+'/'

#set up movie writing stuff
# Set up formatting for the movie files
Writer = animation.writers['ffmpeg']
writer = Writer(fps=20, metadata=dict(artist='Me'), bitrate=1800)
# fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
# out = cv2.VideoWriter(outputdir+'!!all_runs_under_'+str(int(collision_thresh))+'.mp4',fourcc, 30, (640,480), 1)

def init():
    ax.add_patch(background)
    ax.add_patch(left_line)
    ax.add_patch(right_line)
    ax.add_patch(center_line_1)
    ax.add_patch(center_line_2)
    ax.add_patch(car_plot)
    ax.add_patch(deer_plot)
    # plot(ayg,axg,'k-')
    fcax.set_xlabel('ay')
    fcax.set_ylabel('ax')
    # fcax.axis('equal')
    theta = arange(0,2*3.1415,0.01)
    x = 0.5*cos(theta)
    y = 0.5*sin(theta)
    fcax.plot(x,y,'r--')
    return car_plot,deer_plot,

# Set animation
def animate(i):
    #friction circle plot
    fcax.clear()
    theta = arange(0,2*3.1415,0.01)
    x = 1.*cos(theta)
    y = 1.*sin(theta)
    fcax.set_xlabel('ay (g)')
    fcax.set_ylabel('ax (g)')
    fcax.set_title('Acceleration')
    # fcax.plot(ayg,axg,'r--')
    fcax.plot(ayg[3*i],axg[3*i],'ro',ms=15)
    fcax.plot(x,y,'r--')
    

    #axis('equal')
    #car plot
    ax.set_title('Run '+str(filename[7:-4])+' min distance: '+str(min(distance))+'m')
    car_plot.set_width(car_length)
    car_plot.set_height(car_width)
    car_plot.set_xy([car_x[i*3]-(car_length/2), car_y[i*3]-(car_width/2)])
    car_plot.angle = car_yaw[i*3]*180/3.14
    #deer plot
    deer_plot.set_width(deer_length)
    deer_plot.set_height(deer_width)
    deer_plot.set_xy([deer_x[i*3]-(deer_length/2*sin(deer_yaw[i*3])), deer_y[i*3]]-(deer_width/2*cos(deer_yaw[i*3])))
    deer_plot.angle = 90-deer_yaw[i*3]*180/3.14

    #steering wheel plot
    R = .15#meters, steering wheel radius
    ratio = 10.0#ratio of road wheel angle to steering wheel angle
    xb = R*cos(theta)
    yb = R*sin(theta)
    L1x = R*cos(pi/2+actual_steervec[i*3]*ratio)
    L1y = R*sin(pi/2+actual_steervec[i*3]*ratio)
    L2x = R*cos(pi/2+actual_steervec[i*3]*ratio+2.0*pi/3)
    L2y = R*sin(pi/2+actual_steervec[i*3]*ratio+2.0*pi/3)
    L3x = R*cos(pi/2+actual_steervec[i*3]*ratio+4.0*pi/3)
    L3y = R*sin(pi/2+actual_steervec[i*3]*ratio+4.0*pi/3)
    wax.clear()
    wax.set_title('Steering Wheel')
    wax.plot(xb,yb,'k',linewidth=5)
    wax.plot([0,L1x],[0,L1y],'r',linewidth=5)
    wax.plot([0,L2x],[0,L2y],'k',linewidth=5)
    wax.plot([0,L3x],[0,L3y],'k',linewidth=5)
    wax.set_xlim([-.3,.3])
    wax.set_ylim([-.3,.3])

    #axis('equal')
    


    return car_plot,deer_plot,

for i, filename in enumerate(sort(os.listdir(outputdir))):
    if 'simout' in filename:
        print filename[7:]
        #this is the droid we're looking for. Load data from this run
        cardata = loadtxt(outputdir+filename)
        #pull car states
        carx = cardata[:,3:9]
        #pull out car position
        car_x = carx[:,2]
        car_y = carx[:,0]
        #pull out deer states
        deerx = cardata[:,15:]
        #pull out deer position
        deer_x = deerx[:,2]
        deer_y = deerx[:,3]
        #calculate distance
        distance = sqrt((car_x-deer_x)**2+(car_y-deer_y)**2)
        if min(distance)<collision_thresh:
            t = cardata[:,0]
            command_steervec = cardata[:,1]
            actual_steervec = cardata[:,2]
            #load variables in
            carxdot = cardata[:,9:15]
            ayg = (carxdot[:,1]+carx[:,5]*carx[:,3])/9.81
            axg = (carxdot[:,3]+carx[:,5]*carx[:,1])/9.81
            # figure()
            # plot(t,axg,'ko')
            # show()
            # Define parameters
            deer_length = 1.5 # meters
            deer_width = 0.5 # meters
            car_length = 4.5 # meters
            car_width = 2.0 # meters
            # Create car vectors to be used
            car_yaw = carx[:,4]

            # Create deer vectors to be used
            deer_yaw = deerx[:,1]

            # Create figure
            fig = plt.figure(figsize = (8,8))
            ax = fig.add_subplot(211)
            fcax = fig.add_subplot(223)
            wax = fig.add_subplot(224)
            #plt.axis('equal')
            ax.set_xlim(0, 120)
            ax.set_ylim(-20, 20)
            

            # Initialize rectangles
            thic = 0.25
            car_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='b', alpha = 0.9)
            deer_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='r', alpha = 0.9)
            background = patches.Rectangle((-100,-120),400,200,fc=(.85,.85,.85))
            center_line_1 = patches.Rectangle((-10,(1.75+0.025)),200.0,thic,fc='y')
            center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),200.0,thic,fc='y')
            right_line = patches.Rectangle((-10,-1.75),200,thic,fc='w')
            left_line = patches.Rectangle((-10,4.75),200,thic,fc='w')
            # Run anumation
            downsample = 3
            anim = animation.FuncAnimation(fig, animate,init_func=init,frames=len(car_x)/downsample,interval=1,blit=False)
            vfname = outputdir+'!_run'+filename[7:-4]+'.mp4'
            anim.save(vfname, writer=writer)
            print("wrote video: "+str(filename[0:-4]))
            # print("adding to overall video...")
            # cap = cv2.VideoCapture(vfname)
            # while(cap.isOpened()):
            #     ret, frame = cap.read()
            #     if frame is None:
            #         print "done."
            #         cap.release()
            #     out.write(frame)
out.release()





