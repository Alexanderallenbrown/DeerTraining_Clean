import sys
sys.path.append('../')
from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from BinaryConversion import *
from CV_Deer import *
from Deer_Map import *
from KinCar import KinCar
import os

from MPC_F_braking_KinCar import MPC_Fb

def runSim(speed, angle):

    deer = CV_Deer(Speed_Deer = speed, Psi_Deer = angle)

    # Indicate deer initial position
    deer.x_Deer = 50.0 #sightDistance+car_x_offset

    deer.y_Deer = -2.0#PUTrsion(deer_ind)
        
    # Define simulation time and dt

    t = arange(0,simtime,dt) #takes min, max, and timestep\


    car = BicycleModel(dT = dt, U = setSpeed,tiretype='pacejka')


     #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
    carx = zeros((len(t),len(car.x)))
    carxdot = zeros((len(t),len(car.x)))
    car.x[3] = setSpeed
    car.x[0] = -0.0 #let the vehicle start away from lane.
    car.x[2] = car_x_offset #put the car not at the road start if needed.
    carx[0,:] = car.x

    #initialize for deer as well
    deerx = zeros((len(t),4))
    #fill in initial conditions because they're nonzero
    deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])
    
    MPC = MPC_Fb(q_lane_error =qle,q_obstacle_error =qoe,q_lateral_velocity=qlv,q_steering_effort=qse,q_accel = qa,predictionmethod=predm)

    actual_steervec = zeros(len(t))
    command_steervec = zeros(len(t))
    cafvec = zeros(len(t))
    carvec = zeros(len(t))
    distancevec = zeros(len(t))
    opt_steer = 0
    last_steer_t = 0
    ax = zeros(len(t))
    ay = zeros(len(t))
    gas = 0
    brake = 0

    FileName =outputdir+'speed' + str(speed) + '_angle' + str(angle) +'.txt';
    predFileName = outputdir+'predictions_'+'speed' + str(speed) + '_angle' + str(angle) +'.txt';

    newFile = open(FileName,'w+');
    newFile.close();
    newFile = open(FileName, 'a');
    predF = open(predFileName,'a')

    #now simulate!!
    for k in range(1,len(t)):

            #print carx[k-1,:]
            #print deerx[k-1,:]

            if ((deer.x_Deer - car.x[2]) < swerveDistance): 
                if ((t[k]- last_steer_t) >= MPC.dtp):
                    opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
                    brake = MPC.calcBraking(carnow = car)
                    gas = 0

                    last_steer_t = t[k]
                    #print t[k]
            else:
                opt_steer = 0
                gas = 0
                brake = 0


            #print opt_steer
            carx[k,:],carxdot[k,:],actual_steervec[k] = car.rk_update(gas = gas, brake = brake, steer = opt_steer, cruise = 'off')
            cafvec[k] = car.Caf
            carvec[k] = car.Car
            #deerx[k,:] = array([deer.Speed_Deer, deer.Psi_Deer, deer.x_Deer, deer.y_Deer])#updateDeer(car.x[2])
            deerx[k,:] = deer.updateDeer(car.x[2])
            command_steervec[k] = opt_steer
            distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)


            ay[k] = carxdot[k,1]+carx[k,3]*carx[k,5]
            ax[k] = carxdot[k,3]-carx[k,1]*carx[k,5]

            #print round(t[k],2),round(opt_steer,2),round(deer.y_Deer,2)

            #print(t[k])
            newFile.write(str(t[k])+'\t')
            newFile.write(str(command_steervec[k])+'\t')
            newFile.write(str(actual_steervec[k])+'\t')
            for ind2 in range(0,6):
                newFile.write(str(carx[k,ind2]) + ' \t');
            for ind2 in range(0,6):
                newFile.write(str(carxdot[k,ind2]) + ' \t');
            for ind2 in range(0,4):
                    newFile.write(str(deerx[k,ind2]) + '\t');
            newFile.write('\n')

            predF.write(str(t[k])+'\t')
            for ind2 in range(0,len(MPC.XYPrediction)):  
                predF.write(str(MPC.XYPrediction[ind2])+'\t')
            for ind2 in range(0,len(MPC.XYDeerPrediction)):
                predF.write(str(MPC.XYDeerPrediction[ind2])+'\t')
            predF.write('\n')

    newFile.close()
    predF.close()
    #print distancevec[1:]
    return min(distancevec[1:])

for trial in range(1,2):
    print trial
    for speed in range(6,19):
        for angle in range(-90,91):
            angle_rad = angle*3.1415/180.0
            #basic simulation parameters
            simtime = 10
            dt = 0.01

            numtrials = 100 #how many times do you want to simulate this interaction?
            setSpeed = 25.0
            swerveDistance = 80.0
            sightDistance = 80.0
            car_x_offset = 0 #where are we on the map


            #set up the parameters for the MPC
            qle,qoe,qlv,qse,qa,predm = 10,0,1,1,.005,'CV'


            #where are these runs going to be stored?
            basedir = 'CV_Deer'


            #what kind of driver is this?
            controllertype = 'MPC_F_braking_KinCar'

            #is this sam's deer, or the map deer?
            deertype = 'CV_Deer'

            #where are the outputs going?
            outputdir=basedir+'/'+controllertype+'_'+str(int(qle))+'_'+str(int(qoe))+'_'+str(int(qlv))+'_'+str(int(qse))+'_'+str(int(qa))+'_'+str(predm)+'_sight_'+str(int(sightDistance))+'_swerve_'+str(int(swerveDistance))+'/'+'speed_'+str(speed)+'/'+'angle_'+str(angle)+'/'+'trial_'+str(trial) 
            #if the proper place for these data doesn't exist, create it.
            if not os.path.isdir(outputdir):
                os.makedirs(outputdir)

            distfilename = basedir+'/'+controllertype+'_'+str(int(qle))+'_'+str(int(qoe))+'_'+str(int(qlv))+'_'+str(int(qse))+'_'+str(int(qa))+'_'+str(predm)+'_sight_'+str(int(sightDistance))+'_swerve_'+'distances.txt'
            
            distance = runSim(speed = speed, angle = angle_rad)
            print  str(speed)+ ' \t' + str(angle) + ' \t' + str(distance)

            distF = open(distfilename,'a')
            distF.write(str(speed)+ ' \t' + str(angle) + ' \t' + str(distance) + '\r\n')
            distF.close()

