import sys
from numpy import *
from BicycleModel import *
from matplotlib.pyplot import *
from Driver import *
from RayCasting import *

class Deer_Escape:

    def __init__(self, Psi1_Deer = -.307, y_init = -2.0, tturn_Deer = 0.594, Vmax_Deer = 13.5, Tau_Deer = 2.203, dT = 1./60, x_Deer = 80):
       
        self.Tau_Deer = Tau_Deer  
        self.Vmax_Deer = Vmax_Deer 
        self.tturn_Deer = tturn_Deer
        self.y_init = y_init
        self.Psi1_Deer = Psi1_Deer
        self.dT = dT
        self.x_Deer = x_Deer
        self.y_Deer = y_init
        self.xdot_Deer = 0.
        self.ydot_Deer = 0.
        self.x_StartDeer = 60.
        self.Speed_Deer = 0.
        self.Psi_Deer = self.Psi1_Deer
        self.Amax_Deer = 0.632*Vmax_Deer/Tau_Deer
        self.tmove_Deer = 0.
        self.chPsi = True
        self.turn = False
        self.Vturn_Deer = 0.
        self.Psi2_Deer = 0.


    

    def updateDeer(self,x_Car,y_Car):

        if (self.x_Deer - x_Car) > self.x_StartDeer:
            pass

        else:

            if self.tmove_Deer < self.tturn_Deer:
                self.Psi_Deer = self.Psi1_Deer
                self.Speed_Deer += self.dT/(self.Tau_Deer)*(self.Vmax_Deer-self.Speed_Deer)

                # Vstop_Deer = self.Amax_Deer*(self.tturn_Deer-self.tmove_Deer)+self.Vturn_Deer
                

                # if (self.Vmax_Deer*(1-exp(-self.tmove_Deer/self.Tau_Deer)) < Vstop_Deer):
                #     self.Speed_Deer += self.dT/(self.Tau_Deer)*(self.Vmax_Deer-self.Speed_Deer)

                # else:
                #     self.Speed_Deer = self.Speed_Deer - self.Amax_Deer*self.dT

            else:
                if self.chPsi == True:
                    #print "Choose Psi2"
                    self.Psi2_Deer = self.choosePsi2(x_Car,y_Car)
                    self.chPsi = False

                if self.turn == False:
                    if (self.Speed_Deer > self.Vturn_Deer):
                        self.Speed_Deer = self.Speed_Deer - self.Amax_Deer*self.dT

                    else:
                        #print "TURN"
                        self.Psi_Deer = self.Psi2_Deer
                        self.turn = True

                if self.turn == True:
                    self.Speed_Deer += self.dT/(self.Tau_Deer)*(self.Vmax_Deer-self.Speed_Deer)

            self.tmove_Deer += self.dT

        self.xdot_Deer = self.Speed_Deer*sin(self.Psi_Deer)
        self.ydot_Deer = self.Speed_Deer*cos(self.Psi_Deer)

        self.x_Deer += self.dT * self.xdot_Deer
        self.y_Deer += self.dT * self.ydot_Deer

        

        self.xdeer = array([self.Speed_Deer, self.Psi_Deer, self.x_Deer, self.y_Deer])
        
        #print self.xdeer
        return self.xdeer

    def choosePsi2(self,x_Car,y_Car, mean = 135., std = 15.):
        sigmaPsi = random.normal(mean,std,1)
        sigmaPsi = sigmaPsi *3.1416/180.

        carAngle = arctan((self.y_Deer-y_Car)/(self.x_Deer-x_Car))

        sigmaPsi = 3.1415 - sigmaPsi

        if self.y_Deer > y_Car:
            sigmaPsi = sigmaPsi + carAngle

        else:
            sigmaPsi = -sigmaPsi + carAngle

        sigmaPsi = (90*3.1416/180.) - sigmaPsi

        self.Psi2_Deer = sigmaPsi*3.1415/180.
        self.Psi2_Deer = self.Psi2_Deer[0]
        self.Psidot_Deer = abs(self.Psi1_Deer - self.Psi2_Deer)/(10*self.dT)
        self.Vturn_Deer = abs(self.Amax_Deer/self.Psidot_Deer)

        # print "THIS IS THE DATA"
        # print self.Psi2_Deer
        # print self.Psi1_Deer
        # print self.Psidot_Deer
        # print self.Vturn_Deer

        return sigmaPsi



if __name__=='__main__':

    #set up our deer

    deer = Deer_Escape(Psi1_Deer = -0.5, y_init = -30.0, tturn_Deer = 2.5, Vmax_Deer = 18.0, Tau_Deer = .75)
    deer.x_Deer = 80.0

    simtime = 6
    dt = deer.dT
    t = arange(0,simtime,dt) #takes min, max, and timestep

    #now set up the car's parameters
    car = BicycleModel(dT=dt)
    car.x[3] = 20
    steervec = zeros(len(t))

    #set up the driver
    driver = Driver(dt = dt)
    drive = zeros(3)

    #initialize matrices to hold simulation data
    #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

    #now simulate!!
    for k in range(1,len(t)):

        carx_now = carx[k-1,:]
        #print carx_now

        drive[:] = driver.driving(carx = carx_now, deer_x = deerx[k-1,2], setSpeed = 20, brake = 'on', yr = 0.0)

        carx[k,:],junk1,junk2=car.rk_update(brake = drive[1], gas = drive[0], steer = drive[2], cruise = 'off')
        deerx[k,:] = deer.updateDeer(car.x[2],car.x[0])


    distance = sqrt((carx[:,2]-deerx[:,2])**2+(carx[:,0]-deerx[:,3])**2)

    #now plot stuff
    figure()
    plot(carx[:,2],carx[:,0],'ko',deerx[:,2],deerx[:,3],'ro')
    xlabel('x')
    ylabel('y')

    figure()
    subplot(2,1,1)
    plot(t,deerx[:,2])
    subplot(2,1,2)
    plot(t,deerx[:,3])

    deermoving_index = nonzero((deerx[:,2]-carx[:,2])<=deer.x_StartDeer)
    deermoving_index = deermoving_index[0][0]-1
    turn_index = nonzero(t>=(deer.tturn_Deer+t[deermoving_index]))
    turn_index = turn_index[0][0]

    figure()
    subplot(2,1,1)
    plot(t,deerx[:,0],t[deermoving_index],deerx[deermoving_index,0],'ro',t[turn_index],deerx[turn_index,0],'bo')
    subplot(2,1,2)
    plot(t,carx[:,2],t[deermoving_index],carx[deermoving_index,2],'ro')

    figure()
    plot(t, carx[:,3])

    #print(deer.Psi1_Deer,deer.Psi2_Deer)
    #print(deer.Vturn_Deer,deer.Vmax_Deer)

    figure()
    plot(t,distance)


    ### CREATE ANIMATION
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib import animation

    # Define parameters
    deer_length = 1.5 # meters
    deer_width = 0.5 # meters
    car_length = 4.5 # meters
    car_width = 2.0 # meters

    # Create car vectors to be used
    car_x = carx[:,2]
    car_y = carx[:,0]
    car_yaw = carx[:,4]

    # Create deer vectors to be used
    deer_x = deerx[:,2]
    deer_y = deerx[:,3]
    deer_yaw = deerx[:,1]
    deer_vel = deerx[:,0]

    # Create figure
    fig = plt.figure(facecolor = 'black')
    ax = fig.add_subplot(111, facecolor = 'black')
    #plt.axis('equal')
    #ax.set_ylim(-25, 25)
    ax.set_xlim([10.0, 110.0])
    ax.set_ylim([-20.0,20.0])
    ax.set_aspect('equal')

    # vel_plot = fig.add_subplot(211, facecolor = 'black')
    # vel_plot.plot(t,deer_vel, linewidth = 3)
    # vel_plot.spines['left'].set_color('white')
    # vel_plot.spines['bottom'].set_color('white')
    # vel_plot.tick_params(axis='x', colors='white', size=10)
    # vel_plot.tick_params(axis='y', colors='white', size=10)
    # xlabel('Time (s)',fontsize=20)
    # ylabel('Velocity (m/s)',fontsize=20)
    # vel_plot.yaxis.label.set_color('white')
    # vel_plot.yaxis.label.set_size(20)
    # vel_plot.xaxis.label.set_color('white')
    # vel_plot.yaxis.label.set_size(20)



    # Initialize rectangles
    car_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='b', alpha = 1)
    deer_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='r', alpha = 1)
    background = patches.Rectangle((-100,-100),400,400,fc='k')
    center_line_1 = patches.Rectangle((-10,(1.75+0.025)),200,0.1,fc='y')
    center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),200,0.1,fc='y')
    right_line = patches.Rectangle((-10,-1.75),200,0.1,fc='w')
    left_line = patches.Rectangle((-10,4.75),200,0.1,fc='w')
    #vel_circle = patches.Circle((0,0),radius=0.1, fc='r')
    #particle, = vel_plot.plot([], [], 'ro', ms=10)


    def init():
        ax.add_patch(car_plot)
        ax.add_patch(deer_plot)
        ax.add_patch(background)
        ax.add_patch(left_line)
        ax.add_patch(right_line)
        ax.add_patch(center_line_1)
        ax.add_patch(center_line_2)
        #particle.set_data([], [])
        return car_plot,deer_plot,#particle,


    # Set animation
    def animate(i):
        car_plot.set_width(car_length)
        car_plot.set_height(car_width)
        car_plot.set_xy([car_x[i]-(car_length/2), car_y[i]-(car_width/2)])
        car_plot.angle = car_yaw[i]*180/3.14

        deer_plot.set_width(deer_length)
        deer_plot.set_height(deer_width)
        deer_plot.set_xy([deer_x[i]-(deer_length/2*sin(deer_yaw[i])), deer_y[i]]-(deer_width/2*cos(deer_yaw[i])))
        deer_plot.angle = 90-deer_yaw[i]*180/3.14

        #particle.set_data(t[i], deer_vel[i])

        return car_plot,deer_plot,#particle,



    # Run anumation
    anim = animation.FuncAnimation(fig, animate,init_func=init,frames=len(car_x),interval=10,blit=True)
    Writer = animation.writers['imagemagick']
    writer = Writer(fps=15, metadata=dict(artist='Me'),bitrate=1800)
    #anim.save('deer.gif',writer=writer)

    ### ANIMATION END


    show()