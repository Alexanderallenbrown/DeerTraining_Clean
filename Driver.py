import sys
from numpy import *
from BicycleModel import *
from matplotlib.pyplot import *

# Car order is y,v,x,u,psi,r

class Driver:

    def __init__(self, dt, L = 20, cruise_gain = 5, steer_gain = 0.01, brakeCount = 1):

        self.L = L # Preview length in meters
        self.dt = dt
        self.cruise_gain = cruise_gain
        self.steer_gain = steer_gain
        self.brakeCount = brakeCount

    def autoSwerve(self, carx, yr = 0):

        yc = carx[0]
        fi = carx[4]
        ep = yc + self.L*sin(fi)
        turn = self.steer_gain*(yr-ep)

        steer = turn

        return steer

    def cruise(self, setSpeed, carx):
        gas = self.cruise_gain*(setSpeed-carx[3]) #Just p-control for the moment TODO
        
        if gas<0:
            gas = 0

        else:
            pass

        return gas

    # def brake(self, brakeTime = 3):

    #     braking = True
    #     brakeCount = 0

    #     if(braking == True):
    #         brakeCount = brakeCount + 1


    #     if(brakeCount < brakeTime/self.dt and braking == True): #integer is number of seconds to brake for
    #         brake = 1
    #         gaspower = 0
    #     else:
    #         brake = 0
    #         braking = False

    #     return brake

    def driving(self,carx, deer_x, setSpeed, brake = 'off', yr = 0, brakeTime = 3, brakeDistance = 20, swerveDistance = 50): 

        brakePower = 1

        if ((deer_x - carx[2]) > swerveDistance): 

            steer = self.autoSwerve(carx = carx, yr = 0)

        if ((deer_x - carx[2]) <= swerveDistance): 

            steer = self.autoSwerve(carx = carx, yr = yr)

        if ((deer_x - carx[2]) > brakeDistance):
        
            gas = self.cruise(setSpeed = setSpeed, carx = carx)
            brake = 0

        if ((deer_x - carx[2]) <= brakeDistance):

            if brake == 'on':

                if(self.brakeCount < (brakeTime/self.dt)):
                    gas = 0
                    brake = brakePower
                    self.brakeCount = self.brakeCount + 1    

                else:
                     gas = self.cruise(setSpeed = setSpeed, carx = carx)
                     brake = 0

            else:
                gas = self.cruise(setSpeed = setSpeed, carx = carx)
                brake = 0

        return array([gas,brake,steer])





