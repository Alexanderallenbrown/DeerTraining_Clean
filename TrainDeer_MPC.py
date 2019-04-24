from numpy import *
from matplotlib.pyplot import *
from BicycleModel import *
from scipy.optimize import minimize
import operator;
from TraitResultObject import TraitResult;
from GenDevelopment import *;
import sys
from Deer import *
import time
import copy
from MPC_F import *
from MPC_G import *
from MPC_H import *

def BinaryConversion(ind):

    resolution = 5

    # Set minimum and maximum values

    Psi0_min = -3.14/2 # radians
    Psi0_max = 3.14/2 # radians

    SigmaPsi_min = 0 # radians
    SigmaPsi_max = 0.45*3.24 # radians

    tturn_min = 0.4 # seconds
    tturn_max = 2.5 # seconds

    Vmax_min = 5 # m/s
    Vmax_max = 18 # m/s

    Tau_min = 0.75 # seconds
    Tau_max = 5 # seconds

    # Divide individual into different binary 
    Psi0_bin = ind[0:resolution]
    SigmaPsi_bin = ind[resolution:2*resolution]
    tturn_bin = ind[2*resolution:3*resolution]
    Vmax_bin = ind[3*resolution:4*resolution]
    Tau_bin = ind[4*resolution:5*resolution]

    # Convert from binary to decimala
    Psi0 = Psi0_min + (Psi0_max - Psi0_min)*float(int(Psi0_bin,2))/((2**resolution)-1)
    SigmaPsi = SigmaPsi_min + (SigmaPsi_max - SigmaPsi_min)*float(int(SigmaPsi_bin,2))/((2**resolution)-1)
    tturn = tturn_min + (tturn_max - tturn_min)*float(int(tturn_bin,2))/((2**resolution)-1)
    Vmax = Vmax_min + (Vmax_max - Vmax_min)*float(int(Vmax_bin,2))/((2**resolution)-1)
    Tau = Tau_min + (Tau_max - Tau_min)*float(int(Tau_bin,2))/((2**resolution)-1)

    #Rrint results
    # print(Psi0)
    # print(SigmaPsi)
    # print(tturn)
    # print(Vmax)
    # print(Tau)

    return array([Psi0,SigmaPsi,tturn,Vmax,Tau])

def TestDeer_MPC(deer_ind, n, agent):

    min_distance = zeros(n)

    for k_1 in range(0,n):

        MPCDistance = 50.0
        setSpeed = 25.0

        # Where n is the number of drivers we are goin to test each deer against

        deer = Deer(Psi0_Deer = deer_ind[0], Sigma_Psi = deer_ind[1], tturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

        # Indicate deer initial position
        deer.x_Deer = 80
        deer.y_Deer = -2
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
            MPC = MPC_F(q_lane_error = weight,q_obstacle_error =1.0/weight*2,q_lateral_velocity=0.00,q_steering_effort=0.0,q_accel = 0.005,predictionmethod='CV')
  
            for k in range(1,len(t)):

                if ((deer.x_Deer - car.x[2]) < MPCDistance): 
                    opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
                else:
                    opt_steer = 0

                carx[k,:],junk = car.heuns_update(steer = opt_steer, setspeed = 25.0)
                deerx[k,:] = deer.updateDeer(car.x[2])
                distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)

            distancevec = distancevec[1:len(distancevec)]
            min_distance[k_1] = min(distancevec)


        if agent == "G":
            MPC = MPC_G(q_obstacle_error = 1000000000.0,q_x_accel=0.0,q_cruise_speed=0.01,brake_max = 0.5,predictionmethod='CV')
        
            for k in range(1,len(t)):

                opt_steer = 0
                gas,brake = MPC.calcOptimal(carnow = car, deernow = deer, setSpeed = setSpeed)

                if ((sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2) < MPCDistance) and (deer.x_Deer>car.x[2])):

                    carx[k,:],junk = car.heuns_update(gas = gas, brake = brake, steer = 0, cruise = 'off')
                    deerx[k,:] = deer.updateDeer(car.x[2])

                else:
                    carx[k,:],junk = car.heuns_update(steer = 0, setspeed = setSpeed,)
                    deerx[k,:] = deer.updateDeer(car.x[2])

                distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)

            distancevec = distancevec[1:len(distancevec)]
            min_distance[k_1] = min(distancevec)

        if agent == "H":
            MPC = MPC_H(q_lane_error = 10.0,q_obstacle_error_F = .05,q_lateral_velocity = 0.00,q_steering_effort = 0.0,q_lat_accel = 0.005,q_obstacle_error_G = 1000000000.0, q_x_accel = 0.0, q_cruise_speed = 25.0, gas_max = 0.25, brake_max = 0.25, predictionmethod = 'CV')
    
            for k in range(1,len(t)):
                
                gas,brake,steer = MPC.calcOptimal(carnow = car, deernow = deer, setSpeed = setSpeed)

                if ((deer.x_Deer - car.x[2]) < MPCDistance):

                    carx[k,:],junk = car.heuns_update(gas = gas, brake = brake, steer = steer, cruise = 'off')
                    deerx[k,:] = deer.updateDeer(car.x[2])


                else:
                    carx[k,:],junk = car.heuns_update(steer = steer, setspeed = setSpeed,)
                    deerx[k,:] = deer.updateDeer(car.x[2])

                distancevec[k] = sqrt((deer.x_Deer - car.x[2])**2+(deer.y_Deer - car.x[0])**2)

            distancevec = distancevec[1:len(distancevec)]
            min_distance[k_1] = min(distancevec)

    # Calculate IQM

    # Sort values from smallest to largest
    min_distance = sorted(min_distance)
    # Eliminate lower and upper quartiles
    min_distance = min_distance[2:6]
    # Calculate the IQM
    avg_min_distance = mean(min_distance)
    # print(avg_min_distance)

    return(avg_min_distance)

def demo():

    for generation_number in range(1,100):

        for driver_type in range(1,4):

            if driver_type == 1:
                agent_type = "F"

            if driver_type == 2:
                agent_type = "G"

            if driver_type == 3:
                agent_type = "H"

            print "New generation"

            print generation_number

            generation = generation_number
            agent = agent_type
            #Select agent:
            # A = Human
            # B = Straight
            # C = Swerve
            # D = Brake
            # E = Hybrid

            print generation

            Gfname = 'GenerationFiles/generations' + str(agent) + '/Generation' + str(generation) + '.txt';

            print Gfname

            intermediatePopulationSize = 10;
            numberOfHumans = 8;
            populationSize = 15;

            n = populationSize
            m = intermediatePopulationSize;
            h = numberOfHumans;

            #should have an array of size m*h (of object values )

            #in some way, read in a text file to fill an array
            with open(Gfname, "r") as ins:
                CurrentGenarray = []
                for line in ins:
                    values = line.split()
                    deer = TraitResult();
                    deer.assign(str(values[0]),float(values[1]));
                    CurrentGenarray.append(deer)

            # we now have an arrary of deer objects, paired values of attributes and the corresponding results
            CurrentGenarray.sort(key=operator.attrgetter("result"))

            print "This is the CurrentGenarray"

            print CurrentGenarray

            for x in range(0, len(CurrentGenarray)):
                print CurrentGenarray[x].traits

            for x in range(0, len(CurrentGenarray)):
                print CurrentGenarray[x].result

            NewInterGenArray = [];

            print "This is the CurrentGenarray again"

            print CurrentGenarray

            for x in range(0, m):
                develMethod = random.randint(6);
                if develMethod == 0:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do single point cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = SinglePoint(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 1:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do double point cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = DoublePoint(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 2:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do random point cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = RandomPoint(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 3:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do and cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = AndCross(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 4:
                    inds = random.choice(len(CurrentGenarray),2);
                    print str(x) + ' do or cross with ' + str(inds[0]) + ' ' + str(inds[1]);
                    NewDeer = OrCross(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                if develMethod == 5:
                    inds = random.choice(len(CurrentGenarray),1);
                    print str(x) + ' do mutation with ' + str(inds[0]);
                    NewDeer = Mutate(CurrentGenarray[inds[0]]);
                NewInterGenArray.append(NewDeer);

            print '';
            for x in range(0, len(CurrentGenarray)):
                print  str(x) + ' ' + str(CurrentGenarray[x].traits) + ' ' + str(CurrentGenarray[x].result);    
            print '';
            for x in range(0, len(NewInterGenArray)):
                print  str(x) + ' ' + str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result);  
            print '';

            #Test deer in intermediate generation
            for index in range(0,m):
                CurrentDeer = BinaryConversion(str(NewInterGenArray[index].traits))
                print CurrentDeer
                NewInterGenArray[index].result = TestDeer_MPC(CurrentDeer, n, agent)
                print NewInterGenArray[index].result

            for x in range(0, n):
                NewInterGenArray.append(CurrentGenarray[x])

            for x in range(0, len(NewInterGenArray)):
                print str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result);  
            print '';

            #Now, total array of intermediate and base generation, with scores

            NewInterGenArray.sort(key=operator.attrgetter("result"))

            for x in range(0, len(NewInterGenArray)):
                print str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result);  
            print '';

            NewBaseGenArray = []

            for x in range(0, n/2):
                NewBaseGenArray.append(NewInterGenArray[0]);
                NewInterGenArray.pop(0)

            for x in range(0,(n+m)/5):
                NewInterGenArray.pop(len(NewInterGenArray)-1)

            for x in range(0, n/2+1):
                randIndex = random.randint(len(NewInterGenArray))
                NewBaseGenArray.append(NewInterGenArray[randIndex]);
                NewInterGenArray.pop(randIndex);


            for x in range(0, len(NewInterGenArray)):
                print str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result);  
            print '';

            for x in range(0, len(NewBaseGenArray)):
                print str(NewBaseGenArray[x].traits) + ' ' + str(NewBaseGenArray[x].result);    
            print '';

            G2fname = 'GenerationFiles/generations' + str(agent) + '/Generation' + str(generation+1) + '.txt';

            newGenFile = open(G2fname,'w+');
            newGenFile.close();
            newGenFile = open(G2fname, 'a');
            for x in range(0, len(NewBaseGenArray)):
                newGenFile.write(str(NewBaseGenArray[x].traits) + ' ' + str(NewBaseGenArray[x].result) + '\n');

def FirstGen():

    Deer10 = ['1011110011010101111100000','1000011110110111001101000','0011010011101011111001101','1011001010011011110100111','1110001110010110110101000','0101011010101111100110101','1001011110101011000101110','1110100000110001010111001','1011101101011011001011011','0010100010001101001001111','0101111110001101001100001','1001010110101111010110110','0010010000000111000101001','0001000001001100100110000','0101010000110001110001001']
    for agent_ind in range(1,4):    

        if agent_ind == 1:
            agent = 'F'

        if agent_ind == 2:
            agent = 'G'

        if agent_ind == 3:
            agent = 'H'

        print agent

        for ind in range(0,len(Deer10)):

            Deer1 = Deer10[(ind)]

            print str(Deer1)

            Deer1 = BinaryConversion(Deer1)

            print(Deer1)

            Distance1 = TestDeer_MPC(deer_ind=Deer1, n=8, agent = agent)

            print(Distance1)




if __name__=='__main__':
    
    demo()