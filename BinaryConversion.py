import sys
from numpy import *
from BicycleModel import *
from matplotlib.pyplot import *
# from Deer import *

def BinaryConversion_Escape(ind):

    resolution = 5

    # Set minimum and maximum values

    Psi1_min = -3.14/2 # radians
    Psi1_max = 3.14/2 # radians

    yinit_min = -4. # meters
    yinit_max = -30. # meters

    dturn_min = 10 # meters
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

def BinaryConversion(ind):

    resolution = 5

    # Set minimum and maximum values

    Psi0_min = -3.14/2 # radians
    Psi0_max = 3.14/2 # radians

    SigmaPsi_min = 0 # radians
    SigmaPsi_max = 0.45*3.24 # radians

    dturn_min = 0.4 # seconds
    dturn_max = 2.5 # seconds

    Vmax_min = 5 # m/s
    Vmax_max = 18 # m/s

    Tau_min = 0.75 # seconds
    Tau_max = 5 # seconds

    # Divide individual into different binary 
    Psi0_bin = ind[0:resolution]
    SigmaPsi_bin = ind[resolution:2*resolution]
    dturn_bin = ind[2*resolution:3*resolution]
    Vmax_bin = ind[3*resolution:4*resolution]
    Tau_bin = ind[4*resolution:5*resolution]

    # Convert from binary to decimala
    Psi0 = Psi0_min + (Psi0_max - Psi0_min)*float(int(Psi0_bin,2))/((2**resolution)-1)
    SigmaPsi = SigmaPsi_min + (SigmaPsi_max - SigmaPsi_min)*float(int(SigmaPsi_bin,2))/((2**resolution)-1)
    dturn = dturn_min + (dturn_max - dturn_min)*float(int(dturn_bin,2))/((2**resolution)-1)
    Vmax = Vmax_min + (Vmax_max - Vmax_min)*float(int(Vmax_bin,2))/((2**resolution)-1)
    Tau = Tau_min + (Tau_max - Tau_min)*float(int(Tau_bin,2))/((2**resolution)-1)

    #Rrint results
    # print(Psi0)
    # print(SigmaPsi)
    # print(dturn)
    # print(Vmax)
    # print(Tau)

    return array([Psi0,SigmaPsi,dturn,Vmax,Tau])

def TestDeer(deer_ind, n, agent):

	# Where n is the number of humans

	min_distance = zeros(n)

	if agent == "B":
		
		setSpeed = 25
		brake = 'off'
		brakeTime = 0
		yr = 0

	if agent == "C":

		setSpeed = 25
		brake = 'off'
		brakeTime = 0
		yr = 3.5

	if agent == "D":

		setSpeed = 25
		brake = 'on'
		brakeTime = 3
		yr = 0

	if agent == "E":

		setSpeed = 25
		brake = 'on'
		brakeTime = 3
		yr = 3.5


	for k_1 in range(0,n):

		setSpeed = 25.0

		# Where n is the number of drivers we are goin to test each deer against

		deer = Deer(Psi0_Deer = deer_ind[0], Sigma_Psi = deer_ind[1], dturn_Deer = deer_ind[2], Vmax_Deer = deer_ind[3], Tau_Deer = deer_ind[4])

		# Indicate deer initial position
		deer.x_Deer = 80
		deer.y_Deer = -2
	 	# Define simulation time and dt
		simtime = 10
		dt = deer.dT
		t = arange(0,simtime,dt) #takes min, max, and timestep\

	    #now set up the car's parameters		
		car = BicycleModel(dT=dt,U=20)
		steervec = zeros(len(t))

	 	#set up the driver
		driver = Driver(dt = dt)
		drive = zeros(3)

	    #initialize matrices to hold simulation data
	    #car state vector #print array([[Ydot],[vdot],[Xdot],[Udot],[Psidot],[rdot]])
		carx = zeros((len(t),len(car.x)))
		car.x[3] = setSpeed
		carx[0,:] = car.x

	    #initialize for deer as well
		deerx = zeros((len(t),4))
	    #fill in initial conditions because they're nonzero
		deerx[0,:] = array([deer.Speed_Deer,deer.Psi_Deer,deer.x_Deer,deer.y_Deer])

	    #now simulate!!
		for k in range(1,len(t)):

			carx_now = carx[k-1,:]

			drive[:] = driver.driving(carx = carx_now, deer_x = deerx[k-1,2], setSpeed = setSpeed, brake = brake, yr = yr, brakeTime = brakeTime)

			carx[k,:],junk,junk2=car.heuns_update(brake = drive[1], gas = drive[0], steer = drive[2], cruise = 'off')
			deerx[k,:] = deer.updateDeer(car.x[2])

		
		distance = sqrt((carx[:,2]-deerx[:,2])**2+(carx[:,0]-deerx[:,3])**2)

		min_distance[k_1] = min(distance)

	# Calculate IQM

	# Sort values from smallest to largest
	min_distance = sorted(min_distance)
	# Eliminate lower and upper quartiles
	min_distance = min_distance[2:6]
	# Calculate the IQM
	avg_min_distance = mean(min_distance)
	# print(avg_min_distance)

	return(avg_min_distance)

if __name__=='__main__':

	Deer10 = ['0000011111111111111111111','1011110011010101111100000','1011110011010101111100000','1000011110110111001101000','0011010011101011111001101''1011001010011011110100111','1110001110010110110101000','0101011010101111100110101','1001011110101011000101110','1110100000110001010111001','1011101101011011001011011','0010100010001101001001111','0101111110001101001100001','1001010110101111010110110','0010010000000111000101001','0001000001001100100110000','0101010000110001110001001']

	for agent_ind in range(1,5):	

		if agent_ind == 1:
			agent = 'B'

		if agent_ind == 2:
			agent = 'C'

		if agent_ind == 3:
			agent = 'D'

		if agent_ind == 4:
			agent = 'E'

		print agent

		for ind in range(0,len(Deer10)):

		 	Deer1 = Deer10[(ind)]

		 	print str(Deer1)

		 	Deer1 = BinaryConversion(Deer1)

		 	print(Deer1)

		 	Distance1 = TestDeer(deer_ind=Deer1, n=8, agent = agent)

		 	print(Distance1)
