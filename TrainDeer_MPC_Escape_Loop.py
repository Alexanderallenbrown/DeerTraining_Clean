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

<<<<<<< HEAD
    mapa = 'real_tree_wismer'
    xCar = 60
    setSpeed = 15
=======
    mapa = 'nothing'
    xCar = 0
    setSpeed = 20
>>>>>>> 565ffbeb7123072484cd53b33120545c90ee7497
    generation_number = 1

    agent_type = "Fb"
    meanRes = 100.
    collIndex = 0


    intermediatePopulationSize = 10;
    numberOfHumans = 8
    populationSize = 15;
    genomeLength = 25

    n = populationSize
    m = intermediatePopulationSize;
    h = numberOfHumans;

    if generation_number == 1:
        FirstGen(setSpeed,xCar,mapa,h,n)  

    while True:

        while ((collIndex == 0.0) and (generation_number < 21)):

            print('New generation')

            print(generation_number)

            generation = generation_number
            agent = agent_type
            #Select agent:
            # A = Human
            # B = Straight
            # C = Swerve
            # D = Brake
            # E = Hybrid

            print(generation)

            Gfname = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa) + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/Generation' + str(generation) + '.txt';

            print(Gfname)

            #should have an array of size m*h (of object values )

            #in some way, read in a text file to fill an array
            with open(Gfname, "r") as ins:
                CurrentGenarray = []
                for line in ins:
                    values = line.split()
                    deer = TraitResult();
                    resultVec = []
                    for ind in range(4,len(values)):
                        resultVec.append(float(values[ind]))
                    #resultVec = values[3:]
                    print('RESULT VEC')
                    print(resultVec)
                    deer.assign(int(values[0]),str(values[1]),float(values[2]),float(values[3]),resultVec)
                    CurrentGenarray.append(deer)

            # we now have an arrary of deer objects, paired values of attributes and the corresponding results
            CurrentGenarray.sort(key=operator.attrgetter("result"))

            print("This is the CurrentGenarray")

            print(CurrentGenarray)

            for x in range(0, len(CurrentGenarray)):
                print(CurrentGenarray[x].traits)

            for x in range(0, len(CurrentGenarray)):
                print(CurrentGenarray[x].result)

            NewInterGenArray = []


            print("This is the CurrentGenarray again")

            print(CurrentGenarray)

            DevTrials = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa)  + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/trialData/generation' + str(generation+1)

            if not os.path.exists(DevTrials):
                os.makedirs(DevTrials)

            DevTrials = DevTrials + '/GenDevelopment.txt'

            newDevFile = open(DevTrials,'w+');
            newDevFile.close();
            newDevFile = open(DevTrials, 'a');

            # Create and assign deer IDs
            deerID_ind = range(1,m+6)
            deerID = zeros(m+5)
            for ind in range(0,len(deerID)):
                if deerID_ind[ind] < 10:
                    deerID[ind] = str(generation+1) + '0' + str(deerID_ind[ind])
                else:
                    deerID[ind] = str(generation+1) + str(deerID_ind[ind])
                deerID[ind] = int(deerID[ind])

            for x in range(0, m):
                develMethod = random.randint(6);
                if develMethod == 0:
                    inds = random.choice(len(CurrentGenarray),2);
                    print(str(x) + ' do single point cross with ' + str(inds[0]) + ' ' + str(inds[1]))
                    NewDeer = SinglePoint(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                    newDevFile.write(str(int(deerID[x])) + ' - ' + str(NewDeer.traits) + ': Single point cross with ' + str(CurrentGenarray[inds[0]].traits) + ' (' + str(CurrentGenarray[inds[0]].id) + ') and ' + str(CurrentGenarray[inds[1]].traits) + ' (' + str(CurrentGenarray[inds[1]].id) + ')' )
                    newDevFile.write('\n')
                if develMethod == 1:
                    inds = random.choice(len(CurrentGenarray),2);
                    print(str(x) + ' do double point cross with ' + str(inds[0]) + ' ' + str(inds[1]))
                    NewDeer = DoublePoint(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                    newDevFile.write(str(int(deerID[x])) + ' - ' + str(NewDeer.traits) + ': Double point cross with ' + str(CurrentGenarray[inds[0]].traits) + ' (' + str(CurrentGenarray[inds[0]].id) + ') and ' + str(CurrentGenarray[inds[1]].traits) + ' (' + str(CurrentGenarray[inds[1]].id) + ')' )
                    newDevFile.write('\n')
                if develMethod == 2:
                    inds = random.choice(len(CurrentGenarray),2);
                    print(str(x) + ' do random point cross with ' + str(inds[0]) + ' ' + str(inds[1]))
                    NewDeer = RandomPoint(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                    newDevFile.write(str(int(deerID[x])) + ' - ' + str(NewDeer.traits) + ': Random point cross with ' + str(CurrentGenarray[inds[0]].traits) + ' (' + str(CurrentGenarray[inds[0]].id) + ') and ' + str(CurrentGenarray[inds[1]].traits) + ' (' + str(CurrentGenarray[inds[1]].id) + ') ' )
                    newDevFile.write('\n')
                if develMethod == 3:
                    inds = random.choice(len(CurrentGenarray),2);
                    print(str(x) + ' do and cross with ' + str(inds[0]) + ' ' + str(inds[1]))
                    NewDeer = AndCross(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                    newDevFile.write(str(int(deerID[x])) + ' - ' + str(NewDeer.traits) + ': And cross with ' + str(CurrentGenarray[inds[0]].traits) + ' (' + str(CurrentGenarray[inds[0]].id) + ') and ' + str(CurrentGenarray[inds[1]].traits) + ' (' + str(CurrentGenarray[inds[1]].id) + ')' )
                    newDevFile.write('\n')
                if develMethod == 4:
                    inds = random.choice(len(CurrentGenarray),2);
                    print(str(x) + ' do or cross with ' + str(inds[0]) + ' ' + str(inds[1]))
                    NewDeer = OrCross(CurrentGenarray[inds[0]],CurrentGenarray[inds[1]]);
                    newDevFile.write(str(int(deerID[x])) + ' - ' + str(NewDeer.traits) + ': Or cross with ' + str(CurrentGenarray[inds[0]].traits) + ' (' + str(CurrentGenarray[inds[0]].id) + ') and ' + str(CurrentGenarray[inds[1]].traits) + ' (' + str(CurrentGenarray[inds[1]].id) + ')' )
                    newDevFile.write('\n')
                if develMethod == 5:
                    inds = random.choice(len(CurrentGenarray),1);
                    print(str(x) + ' do mutation with ' + str(inds[0]))
                    NewDeer = Mutate(CurrentGenarray[inds[0]]);
                    newDevFile.write(str(int(deerID[x])) + ' - ' + str(NewDeer.traits) + ': Mutate ' + str(CurrentGenarray[inds[0]].traits) + ' (' + str(CurrentGenarray[inds[0]].id) + ')')
                    newDevFile.write('\n')
                NewInterGenArray.append(NewDeer);
            newDevFile.close()

            for ind in range(0,m):
                NewInterGenArray[ind].id = int(deerID[ind])
            print('DEER IDs')
            print(deerID)

            print('');
            for x in range(0, len(CurrentGenarray)):
                print(str(x) + ' ' + str(CurrentGenarray[x].traits) + ' ' + str(CurrentGenarray[x].result));    
            print('');
            for x in range(0, len(NewInterGenArray)):
                print(str(x) + ' ' + str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result));  
            print('');

            DirTrials = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa)  + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/trialData/generation' + str(generation+1)

            #Test deer in intermediate generation
            for index in range(0,m):
                print("Training deer " + str(index + 1) + " out of " + str(m))
                CurrentDeer = BinaryConversion(str(NewInterGenArray[index].traits))
                print(CurrentDeer)
                NewInterGenArray[index].result,NewInterGenArray[index].collisions,NewInterGenArray[index].resultVec = TestDeer_MPC(CurrentDeer, h, agent, xCar, setSpeed,mapa,DirTrials,int(NewInterGenArray[index].id))
                


                print(NewInterGenArray[index].result)
                print(NewInterGenArray[index].collisions)
                print(NewInterGenArray[index].resultVec)

            for x in range(0, n):
                NewInterGenArray.append(CurrentGenarray[x])

            for x in range(0, len(NewInterGenArray)):
                print(str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result));  
            print('');

            #Now, total array of intermediate and base generation, with scores

            NewInterGenArray.sort(key=operator.attrgetter("result"))

            for x in range(0, len(NewInterGenArray)):
                print(str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result));  
            print('');

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
                print(str(NewInterGenArray[x].traits) + ' ' + str(NewInterGenArray[x].result));  
            print('');

            for x in range(0, len(NewBaseGenArray)):
                print(str(NewBaseGenArray[x].traits) + ' ' + str(NewBaseGenArray[x].result));    
            print('');

            NewBaseGenArray.sort(key=operator.attrgetter("result"))

            # Find the range of results
            resultRange = NewBaseGenArray[n-1].result-NewBaseGenArray[0].result
            print("RESULT RANGE: " + str(resultRange))
            numNewDeer = 5
            minRange = 3

            # If the range is below the minimum, introduce x number of randomly generated deer
            if resultRange < minRange:
                print('Result under minimum: Introduce ' + str(numNewDeer) + ' random deer')
                for ind in range(0,5):
                    newGenomeRandom = randomGenome(genomeLength)
                    print('New Random Deer Genome: ' + str(newGenomeRandom))
                    CurrentDeer = BinaryConversion(newGenomeRandom)
                    NewBaseGenArray[(len(NewBaseGenArray)-1-ind)].traits = str(newGenomeRandom)
                    NewBaseGenArray[(len(NewBaseGenArray)-1-ind)].id = int(deerID[m+ind])
                    NewBaseGenArray[(len(NewBaseGenArray)-1-ind)].result,NewBaseGenArray[(len(NewBaseGenArray)-1-ind)].collisions,NewBaseGenArray[(len(NewBaseGenArray)-1-ind)].resultVec = TestDeer_MPC(CurrentDeer, h, agent, xCar, setSpeed,mapa,DirTrials,str(newGenomeRandom))

            NewBaseGenArray.sort(key=operator.attrgetter("result"))

            G2fname = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa)  + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/Generation' + str(generation+1) + '.txt';

            newGenFile = open(G2fname,'w+');
            newGenFile.close();
            newGenFile = open(G2fname, 'a');

            for x in range(0, len(NewBaseGenArray)):
                newGenFile.write(str(NewBaseGenArray[x].id) + ' ' + str(NewBaseGenArray[x].traits) + ' ' + str(NewBaseGenArray[x].result) + ' ' + str(NewBaseGenArray[x].collisions))
                for ind in range(0,len(NewBaseGenArray[x].resultVec)):
                    newGenFile.write(' ' + str(NewBaseGenArray[x].resultVec[ind]))
                newGenFile.write('\n')
            newGenFile.close()

            # Check for collisions
            collIndex = 0
            for x in range(0, len(NewBaseGenArray)):
                collIndex = collIndex + NewBaseGenArray[x].collisions

            print('NUMBER OF COLLISIONS' + str(collIndex))

            meanRes = NewBaseGenArray[0].result

            #time.sleep(5)

            generation_number = generation_number + 1

        if collIndex > 0:
            print("Crash reduce SPEED")
            print(meanRes)
            setSpeed = setSpeed - 1
            collIndex = 0
            generation_number = 1
            FirstGen(setSpeed,xCar,mapa,h,n)

        else:
            print("We're good move FORWARD")
            print(meanRes)
            setSpeed = 25
            xCar = xCar + 20
            collIndex = 0
            generation_number = 1
            FirstGen(setSpeed,xCar,mapa,h,n)


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

    for k_1 in range(0,n):

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
        
        MPC = MPC_Fb(q_lane_error = weight,q_obstacle_error =0.0/weight*10,q_lateral_velocity=1.0,q_steering_effort=1.0,q_accel = 0.005,predictionmethod='CV')

        # Initialize data saving files
        dirName = Dir + '/ID_' + str(int(deerID))
        FileName = dirName + '/trial_' + str(k_1+1) + '.txt'
        predDirName = Dir + '/preds/ID_' + str(int(deerID))
        predFileName = predDirName + '/trial_' + str(k_1+1) + '.txt'

        if not os.path.exists(dirName):
            os.makedirs(dirName)

        if not os.path.exists(predDirName):
            os.makedirs(predDirName)

        newFile = open(FileName,'w+');
        newFile.close();
        newFile = open(FileName, 'a');
        predF = open(predFileName,'a')


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

                    if ((t[k]- last_steer_t) >= MPC.dtp):
                        opt_steer = MPC.calcOptimal(carnow = car,deernow = deer, yroad = 0)
                        brake = MPC.calcBraking(carnow = car)
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

            predF.write(str(t[k])+'\t')
            for ind2 in range(0,len(MPC.XYPrediction)):  
                predF.write(str(MPC.XYPrediction[ind2])+'\t')
            for ind2 in range(0,len(MPC.XYDeerPrediction)):
                predF.write(str(MPC.XYDeerPrediction[ind2])+'\t')
            predF.write('\n')


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
    predF.close()

    # Calculate an index to know which result corresponds to each trial
    test_number = range(1,11)
    # Create combined vector of 
    results_vec = zip(min_distance,Collision,test_number)
    # Sort the trials by result
    results_vec.sort()

    # Calculate IQM
    # Sort values from smallest to largest
    sorted_min_distance = sorted(min_distance)
    minDistanceVec = min_distance
    # Eliminate lower and upper quartiles
    sorted_min_distance = sorted_min_distance[int(round(n/4.0)):int(ceil(3.0*n/4.0))]
    # Calculate the IQM
    IQM_min_distance = mean(sorted_min_distance)
    # print(avg_min_distance)

    # Determine total number of collisions
    Collisions = sum(Collision[:])

    results_vec_return = []
    for ind in range(0,n):
        results_vec_return.append(results_vec[ind][2])
        results_vec_return.append(results_vec[ind][0])
        results_vec_return.append(results_vec[ind][1])
        # = results_vec_return + str(results_vec[ind][2]) + ' ' + str(results_vec[ind][0]) + ' ' + str(results_vec[ind][1]) + ' '

    print(results_vec_return)
    return IQM_min_distance, Collisions, results_vec_return

def FirstGen(setSpeed, xCar,mapa,h,n):

    Deer10 = []

    for ind in range(0,n):
        Deer10.append(str(randomGenome(25)))

    print('DEER 10')
    print(Deer10)

    #Deer10 = ['1011110011010101111100000','1000011110110111001101000','0011010011101011111001101','1011001010011011110100111','1110001110010110110101000','0101011010101111100110101','1001011110101011000101110','1110100000110001010111001','1011101101011011001011011','0010100010001101001001111','0101111110001101001100001','1001010110101111010110110','0010010000000111000101001','0001000001001100100110000','0101010000110001110001001']
    #Deer10 = ['0000100000011111111100000','0000000000110111001101000','0000000000001011111001101','0000000000011011110100111','0000100000010110110101000','0000100000101111100110101','0000100000101011000101110','0000100000110001010111001','1011101101011011001011011','0010100010001101001001111','0101111110001101001100001','1001010110101111010110110','0010010000000111000101001','0001000001001100100110000','0101010000110001110001001']

    agent = 'Fb'

    directory = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa) + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) 

    if not os.path.exists(directory):
        os.makedirs(directory)

    Gfname = directory + '/Generation1.txt';

    newGenFile = open(Gfname,'w+');
    newGenFile.close();

    DirTrials = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa)  + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/trialData/generation1'

    # Create deer IDs
    deerID = range(101,101+len(Deer10))

    for ind in range(0,len(Deer10)):

        Deer1 = Deer10[(ind)]
        Deer1_bin = Deer1

        print("TRAINING DEER " + str(ind + 1) + " OUT OF " + str(len(Deer10)))

        print("This is the deer we are currently training")
        print(str(Deer1))

        Deer1 = BinaryConversion(Deer1)

        print(Deer1)

        Distance1, Collisions, resultVec = TestDeer_MPC(deer_ind=Deer1, n=h, agent = agent, setSpeed = setSpeed, xCar = xCar, fake_map = mapa, Dir = DirTrials, deerID = int(deerID[ind]))
        print(resultVec)

        

        newGenFile = open(Gfname, 'a');
        newGenFile.write(str(deerID[ind]) + ' ' + str(Deer1_bin) + ' ' + str(Distance1) + ' ' + str(Collisions))
        for ind in range(0,len(resultVec)):
            newGenFile.write(' ' + str(resultVec[ind]))
        newGenFile.write('\n')

        newGenFile.close()


        print("MINIMUM DISTANCE")
        print(str(Deer1_bin))
        print(Distance1)


def randomGenome(genomelength):

    genomeVec = zeros(genomelength)
    genome = ''

    for ind in range(0,genomelength):
        genome = genome + str(random.randint(0,2))

    return str(genome)




if __name__=='__main__':
    demo()

