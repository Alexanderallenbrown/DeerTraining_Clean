import sys
from numpy import *
from BicycleModel import *
from matplotlib.pyplot import *



class CV_Deer:

    def __init__(self,Speed_Deer = 13.5, dT = 1./60, x_Deer = 0., y_Deer = 0., Psi_Deer = 0.):
       
        self.Speed_Deer = Speed_Deer
        self.dT = dT
        self.x_Deer = x_Deer
        self.y_Deer = y_Deer
        self.Psi_Deer = Psi_Deer
        self.xdot_Deer = 0.
        self.ydot_Deer = 0.
        self.x_StartDeer = 60.
        self.xdeer = array([self.Speed_Deer,self.Psi_Deer,self.x_Deer,self.y_Deer])

    def updateDeer(self,x_Car):

        if (self.x_Deer - x_Car) > self.x_StartDeer:
            speed_Deer = 0

        else:
            self.Speed_Deer = self.Speed_Deer

        self.xdot_Deer = self.Speed_Deer*sin(self.Psi_Deer)
        self.ydot_Deer = self.Speed_Deer*cos(self.Psi_Deer)
        #print self.xdot_Deer
        #print self.ydot_Deer

        self.x_Deer += self.dT * self.xdot_Deer
        self.y_Deer += self.dT * self.ydot_Deer

        self.xdeer = array([self.Speed_Deer,self.Psi_Deer, self.x_Deer, self.y_Deer])
        return self.xdeer


def demo():

    #set up our deer
    deer = CV_Deer()
    deer.x_Deer = 80
    deer.y_Deer = -2
    deer.Psi_Deer = -89*3.1415/180


    simtime = 10
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

        drive[:] = driver.driving(carx = carx_now, deer_x = deerx[k-1,2], setSpeed = 20, brake = 'on', yr = -3.5)

        carx[k,:],junk=car.heuns_update(brake = drive[1], gas = drive[0], steer = drive[2], cruise = 'off')
        deerx[k,:] = deer.updateDeer(car.x[2])
        print deerx[k,:]

    distance = sqrt((carx[:,2]-deerx[:,2])**2+(carx[:,0]-deerx[:,3])**2)

    #now plot stuff
    figure()
    plot(carx[:,2],carx[:,0],'ko',deerx[:,2],deerx[:,3],'ro')

    figure()
    subplot(2,1,1)
    plot(t,deerx[:,2])
    subplot(2,1,2)
    plot(t,deerx[:,3])

    figure()
    subplot(2,1,1)
    plot(t,deerx[:,0])
    subplot(2,1,2)
    plot(t,carx[:,2])

    figure()
    plot(t, carx[:,3])

    figure()
    plot(t,distance)


    show()

def TestDeer_MPC_CV(agent):

    #min_distance = zeros((10,181))

    #print min_distance

    print agent

    for speed_ind in range(6,18):

        speed = 1.0*speed_ind

        for angle_ind in range(-90,90):

            angle = 1.0*angle_ind

            MPCDistance = 80.0
            setSpeed = 25.0

            # Where n is the number of drivers we are goin to test each deer against

            # Initiate process
            deer = CV_Deer()
            deer.x_Deer = 80
            deer.y_Deer = -2
            deer.Psi_Deer = angle*3.1415/180
            deer.Speed_Deer = speed
     
            # Define simulation time and dt
            simtime = 10
            dt = deer.dT
            t = arange(0,simtime,dt) #takes min, max, and timestep\

            #now set up the car's parameters        
            car = BicycleModel(dT=dt,U=20)
            
            carx = zeros((len(t),len(car.x)))
            car.x[3] = 25.0
            carx[0,:] = car.x

            #initialize for deer as well
            deerx = zeros((len(t),4))
            #fill in initial conditions because they're nonzero
            deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])
            distancevec = zeros(len(t))

            if agent == "F":

                weight = 10.0
                MPC = MPC_F(q_lane_error = weight,q_obstacle_error =100.0/weight*10,q_lateral_velocity=1.0,q_steering_effort=1.0,q_accel = 0.005,predictionmethod='CV')

                opt_steer = 0
                last_steer_t = 0
                swerveDistance = 80.0

                for k in range(1,len(t)):

                    if ((deer.x_Deer - car.x[2]) < swerveDistance): 
                        if ((t[k]- last_steer_t) >= MPC.dtp):
                            opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
                            last_steer_t = t[k]
                    
             
                    else:
                        opt_steer = 0


                    carx[k,:],junk1,junk2 = car.rk_update(steer = opt_steer, setspeed = 25.0)
                    deerx[k,:] = deer.updateDeer(car.x[2])
                    distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)

                distancevec = distancevec[1:len(distancevec)]

                print str(speed) + ' ' + str(angle) + ' ' + str(min(distancevec))


            if agent == "G":

                MPC = MPC_G(q_obstacle_error = 1000000000.0,q_x_accel=0.0,q_cruise_speed=0.01,brake_max = 1.0,predictionmethod='CV')
                x_acceldistance = 80.0

                for k in range(1,len(t)):

                    opt_steer = 0
                    gas,brake = MPC.calcOptimal(carnow = car, deernow = deer, setSpeed = setSpeed)

                    if ((sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2) < x_acceldistance) and (deer.x_Deer>car.x[2])):

                        carx[k,:],junk,steer = car.rk_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
                        deerx[k,:] = deer.updateDeer(car.x[2])

                    else:
                        carx[k,:],junk,steer = car.rk_update(steer = 0, setspeed = setSpeed,)
                        deerx[k,:] = deer.updateDeer(car.x[2])

                    distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)

                distancevec = distancevec[1:len(distancevec)]
                print str(speed) + ' ' + str(angle) + ' ' + str(min(distancevec))

            if agent == "H":

                MPC = MPC_H(q_lane_error = 10.0,q_obstacle_error_F = 100.0,q_lateral_velocity = 0.10,q_steering_effort = .10,q_lat_accel = 0.005,q_obstacle_error_G = 1000000000.0, q_x_accel = 0.0, q_cruise_speed = 25.0, gas_max = 0.25, brake_max = 0.5, predictionmethod = 'CV')

                last_command_t = -0.1
                steer = 0
                x_acceldistance = 80.0

                for k in range(1,len(t)):
                    opt_steer = 0

                    if ((t[k]- last_command_t) >= 0.1):
                        if ((deer.x_Deer - car.x[2]) > x_acceldistance):
                            steer = 0
                            gas = 0
                            brake = 0

                        else:
                            gas,brake,steer = MPC.calcOptimal(carnow = car, deernow = deer, setSpeed = setSpeed)
                            last_command_t = t[k]

                    else:
                        pass

                    if ((deer.x_Deer - car.x[2]) < x_acceldistance):

                        carx[k,:],carxdot[k,:],actual_steervec[k] = car.rk_update(gas = gas, brake = brake, steer = steer, cruise = 'off')
                        deerx[k,:] = deer.updateDeer(car.x[2])


                    else:
                        carx[k,:],carxdot[k,:], actual_steervec[k] = car.rk_update(steer = steer, setspeed = setSpeed,)
                        deerx[k,:] = deer.updateDeer(car.x[2])

                    distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)

                distancevec = distancevec[1:len(distancevec)]

            if agent == "Fb":
                weight = 10.0
                swerveDistance = 80.0
                last_steer_t = 0
                MPC = MPC_Fb(q_lane_error = weight,q_obstacle_error =200.0/weight*10,q_lateral_velocity=1.0,q_steering_effort=1.0,q_accel = 0.005,predictionmethod='CV')



                for k in range(1,len(t)):

                    if ((deer.x_Deer - car.x[2]) < swerveDistance): 

                        if ((t[k]- last_steer_t) >= MPC.dtp):
                            opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
                            brake = MPC.calcBraking(carnow = car)
                            gas = 0
                            last_steer_t = t[k]
            
                    else:
                        opt_steer = 0
                        gas = 0
                        brake = 0

                    carx[k,:],junk1,junk2 = car.rk_update(gas = gas, brake = brake, steer = opt_steer, cruise = 'off')
                    deerx[k,:] = deer.updateDeer(car.x[2])
                    distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)

                    #print carx[k,:]
                    #print deerx[k,:]
                    #print distancevec[k]


                distancevec = distancevec[1:len(distancevec)]

                #print distancevec


                print str(speed) + ' ' + str(angle) + ' ' + str(min(distancevec))

    return()

if __name__=='__main__':

    for agent_ind in range(4,5):    

        if agent_ind == 1:
            agent = 'F'

        if agent_ind == 2:
            agent = 'G'

        if agent_ind == 3:
            agent = 'H'

        if agent_ind == 4:
            agent = 'Fb'

        print agent

        TestDeer_MPC_CV(agent)