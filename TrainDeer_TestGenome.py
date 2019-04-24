from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
import operator;
from TraitResultObject import TraitResult;
from GenDevelopment import *;
import sys
import time
import copy
from MPC_F_braking_KinCar_Map import *
from Deer_Escape import *
from Crash_Test import CollisionCheck
import os



def demo():

    mapa = 'nothing'

    agent = 'D'

    xCar = 0
    setSpeed = 19

    agent_type = "D"

    n = 100

    deer_ind = '0100011000000001111100100'
    CurrentDeer = BinaryConversion(str(deer_ind))

    DirTrials = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/trialData'

    if not os.path.exists(DirTrials):
        os.makedirs(DirTrials)

    trial_number,min_distance,collision = TestDeer_MPC(CurrentDeer, n, agent, xCar, setSpeed,mapa,DirTrials,int(deer_ind))


    genomeFileName = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/genome.txt'
        
    genomeFile = open(genomeFileName,'w+');
    genomeFile.close();
    genomeFile = open(genomeFileName, 'a');
    genomeFile.write(str(deer_ind))
    genomeFile.close()   


    newFileName = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/results.txt'

    newFile = open(newFileName,'w+');
    newFile.close();
    newFile = open(newFileName, 'a');

    for x in range(0,len(trial_number)):
        newFile.write(str(trial_number[x]) + ' ' + str(min_distance[x]) + ' ' + str(collision[x]))
        newFile.write('\n')
    newFile.close()   


def BinaryConversion(ind):

    resolution = 5

    # Set minimum and maximum values

    Psi1_min = -3.14/2 # radians
    Psi1_max = 3.14/2 # radians

    yinit_min = -4. # meters
    yinit_max = -30. # meters

    dturn_min = 20 # meters
    dturn_max = 120 # meters

    Vmax_min = 5 # m/s
    Vmax_max = 18 # m/s

    Tau_min = 0.75 # seconds
    Tau_max = 5 # seconds

    # Divide individual into different binary 
    Psi1_bin = ind[0:resolution]
    yinit_bin = ind[resolution:2*resolution]
    dturn_bin = ind[2*resolution:3*resolution]
    Vmax_bin = ind[3*resolution:4*resolution]
    Tau_bin = ind[4*resolution:5*resolution]

    # Convert from binary to decimala
    Psi1 = Psi1_min + (Psi1_max - Psi1_min)*float(int(Psi1_bin,2))/((2**resolution)-1)
    yinit = yinit_min + (yinit_max - yinit_min)*float(int(yinit_bin,2))/((2**resolution)-1)
    dturn = dturn_min + (dturn_max - dturn_min)*float(int(dturn_bin,2))/((2**resolution)-1)
    Vmax = Vmax_min + (Vmax_max - Vmax_min)*float(int(Vmax_bin,2))/((2**resolution)-1)
    Tau = Tau_min + (Tau_max - Tau_min)*float(int(Tau_bin,2))/((2**resolution)-1)

    #Rrint results
    # print(Psi1)
    # print(yinit)
    # print(dturn)
    # print(Vmax)
    # print(Tau)

    return array([Psi1,yinit,dturn,Vmax,Tau])

def TestDeer_MPC(deer_ind, n, agent, xCar, setSpeed,fake_map,Dir,deerID):

    min_distance = zeros(n)
    Collision = zeros(n)
    trial_number = zeros(n)

    for k_1 in range(0,n):

        trial_number[k_1] = int(k_1 + 1)

        MPCDistance = 50.0
        setSpeed = setSpeed
        x_car = xCar
        x_deer = xCar + 80.0
        KML = False
        fake_map = fake_map


        print("Run " + str(k_1+1) + " of " + str(n))
        print("The current car speed is " + str(setSpeed) + " m/s, and the starting x is " + str(xCar) + "m")
        print('KML = ' + str(KML) + ', map = ' + str(fake_map))


        # Where n is the number of drivers we are goin to test each deer against

        deer = Deer_Escape_Smooth(Psi1_Deer = deer_ind[0], y_init = deer_ind[1], dturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

        # Indicate deer initial position
        deer.x_Deer = x_deer
        # Define simulation time and dt
        simtime = 10
        dt = deer.dT
        t = arange(0,simtime,dt) #takes min, max, and timestep\

        #now set up the car's parameters        
        car = BicycleModel(dT=dt,U=20)
        
        carx = zeros((len(t),len(car.x)))
        carxdot = zeros((len(t),len(car.x)))
        car.x[3] = setSpeed
        car.x[2] = x_car
        carx[0,:] = car.x

        actual_steervec = zeros(len(t))
        command_steervec = zeros(len(t))

        #initialize for deer as well
        deerx = zeros((len(t),4))
        #fill in initial conditions because they're nonzero
        deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])
        distancevec = zeros(len(t))

        weight = 10.0
        swerveDistance = 50.0
        last_steer_t = -0.1
        deerSight = False
        
        #MPC = MPC_Fb(q_lane_error = weight,q_obstacle_error =0.0/weight*10,q_lateral_velocity=1.0,q_steering_effort=1.0,q_accel = 0.005,predictionmethod='CV')

        # Initialize data saving files
        FileName = Dir + '/trial_' + str(k_1+1) + '.txt'
        # predDirName = Dir + '/preds/ID_' + str(int(deerID))
        # predFileName = predDirName + '/trial_' + str(k_1+1) + '.txt'


        # if not os.path.exists(predDirName):
        #     os.makedirs(predDirName)

        newFile = open(FileName,'w+');
        newFile.close();
        newFile = open(FileName, 'a');
        # predF = open(predFileName,'a')


        for k in range(1,len(t)):

            if car.x[3] > 1.0:

                if deerSight == False:

                    distance_pred = zeros(10)

                    distanceAngle = MapRaycasting([car.x[2],car.x[0]],'mapa',KML = KML, fake = fake_map)

                    deerAngle = arctan2((deer.y_Deer-car.x[0]),(deer.x_Deer-car.x[2]))
                    deerDist = sqrt((deer.y_Deer-car.x[0])**2+(deer.x_Deer-car.x[2])**2)

                    deerAngle = int(deerAngle *180./3.1415)

                    if deerAngle < 0:
                        deerAngle = 360 + deerAngle

                    if (deerDist < distanceAngle[deerAngle]): 
                        deerSight = True

                if deerSight == True:

                    if ((t[k]- last_steer_t) >= 0.1):#MPC.dtp):
                        opt_steer = 0 #MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
                        brake = 0.6 #MPC.calcBraking(carnow = car)
                        gas = 0

                        last_steer_t = t[k]
        
                else:
                    opt_steer = 0
                    gas = 0
                    brake = 0

                carx[k,:],carxdot[k,:],actual_steervec[k] = car.rk_update(gas = gas, brake = brake, steer = opt_steer, cruise = 'off')
                deerx[k,:] = deer.updateDeer(car.x[2],car.x[0])
                distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)
                #print carx[k,:]
                #print deerx[k,:]
                #print distancevec[k]

            else:
                carx[k,:] = array([carx[k-1,0],0.0,carx[k-1,2],0.0,carx[k-1,4],0.0])
                carxdot[k,:] = zeros(6)
                actual_steervec[k] = actual_steervec[k-1]
                deerx[k,:] = array([0.0,deerx[k-1,1],deerx[k-1,2],deerx[k-1,3]])
                distancevec[k] = distancevec[k-1]

            command_steervec[k] = opt_steer

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

            # predF.write(str(t[k])+'\t')
            # for ind2 in range(0,len(MPC.XYPrediction)):  
            #     predF.write(str(MPC.XYPrediction[ind2])+'\t')
            # for ind2 in range(0,len(MPC.XYDeerPrediction)):
            #     predF.write(str(MPC.XYDeerPrediction[ind2])+'\t')
            # predF.write('\n')


        distancevec = distancevec[1:len(distancevec)]
        min_distance[k_1] = min(distancevec)

        Collision[k_1] = bool(0)

        # If the minimum distance is under 2m, check for a collision to occue
        if min_distance[k_1] < 2.0:

            check = CollisionCheck()
            Collision[k_1] = check.collision(carx[:,2],carx[:,0],carx[:,4],deerx[:,2],deerx[:,3],deerx[:,1])
            if Collision[k_1] == True:
                Collision[k_1] = bool(1)
            else:
                Collision[k_1] = bool(0)

        newFile.close();

    return trial_number, min_distance, Collision



if __name__=='__main__':
    demo()

