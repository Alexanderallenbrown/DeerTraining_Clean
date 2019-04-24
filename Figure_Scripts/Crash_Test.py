from numpy import *
from matplotlib.pyplot import *
import os

class CollisionCheck:

    def __init__(self,car_a=1.5,car_b=0.8,car_width=2.0,deer_length=1.5,deer_width=0.5):
        self.car_a = car_a +0.8 #center to back
        self.car_b = car_b +0.8#center to front
        self.car_width = car_width
        self.deer_length = deer_length
        self.deer_width = deer_width

    def rotate(self,x,y,x_rot,y_rot,psi):
        x_new = x_rot + (x-x_rot)*cos(psi) - (y-y_rot)*sin(psi)
        y_new = y_rot + (x-x_rot)*sin(psi) + (y-y_rot)*cos(psi)

        return [x_new,y_new]

    def checkBounds(self,x,y,x_min,x_max,y_min,y_max):
        if ((x>x_min and x<x_max) and (y>y_min and y<y_max)):
            bounds = True
            #print x, x_min, x_max
            #print y, y_min, y_max
        else:
            bounds = False

        return bounds

    def collision(self,car_xpos,car_ypos,car_psi,deer_xpos,deer_ypos,deer_psi):
        Collision = False
        distance = sqrt((car_xpos-deer_xpos)**2+(car_ypos-deer_ypos)**2)
        if min(distance)<2.0:
            for ind in range(0,len(distance)):
                if distance[ind]<2.0:
                    #print 'Under 2m'
                    # car_front = car_xpos[ind]+self.car_b
                    # car_back = car_xpos[ind]-self.car_a
                    # car_left = car_ypos[ind]+0.5*self.car_width
                    # car_right = car_ypos[ind]-0.5*self.car_width

                    # # A: Driver front
                    # carA = [car_xpos[ind] + ((car_front-car_xpos[ind])*cos(car_psi[ind])-(car_left-car_ypos[ind])*sin(car_psi[ind])), car_ypos[ind] + ((car_front-car_xpos[ind])*sin(car_psi[ind])+(car_left-car_ypos[ind])*cos(car_psi[ind]))]
                    # # B: Passanger front
                    # carB = [car_xpos[ind] + ((car_front-car_xpos[ind])*cos(car_psi[ind])-(car_right-car_ypos[ind])*sin(car_psi[ind])), car_ypos[ind] + ((car_front-car_xpos[ind])*sin(car_psi[ind])+(car_right-car_ypos[ind])*cos(car_psi[ind]))]
                    # # C: Passanger rear
                    # carC = [car_xpos[ind] + ((car_back-car_xpos[ind])*cos(car_psi[ind])-(car_right-car_ypos[ind])*sin(car_psi[ind])), car_ypos[ind] + ((car_back-car_xpos[ind])*sin(car_psi[ind])+(car_right-car_ypos[ind])*cos(car_psi[ind]))]
                    # # D: Driver rear
                    # carD = [car_xpos[ind] + ((car_back-car_xpos[ind])*cos(car_psi[ind])-(car_left-car_ypos[ind])*sin(car_psi[ind])), car_ypos[ind] + ((car_back-car_xpos[ind])*sin(car_psi[ind])+(car_left-car_ypos[ind])*cos(car_psi[ind]))]

                    # car = [carA,carB,carC,carD]

                    deer_front = deer_xpos[ind]+0.5*self.deer_length
                    deer_back = deer_xpos[ind]-0.5*self.deer_length
                    deer_left = deer_ypos[ind]+0.5*self.deer_width
                    deer_right = deer_ypos[ind]-0.5*self.deer_width


                    deerA = [deer_xpos[ind] + ((deer_front-deer_xpos[ind])*cos(3.1415/2-deer_psi[ind])-(deer_left-deer_ypos[ind])*sin(3.1415/2-deer_psi[ind])), deer_ypos[ind] + ((deer_front-deer_xpos[ind])*sin(3.1415/2-deer_psi[ind])+(deer_left-deer_ypos[ind])*cos(3.1415/2-deer_psi[ind]))]
                    # B: Passanger front
                    deerB = [deer_xpos[ind] + ((deer_front-deer_xpos[ind])*cos(3.1415/2-deer_psi[ind])-(deer_right-deer_ypos[ind])*sin(3.1415/2-deer_psi[ind])), deer_ypos[ind] + ((deer_front-deer_xpos[ind])*sin(3.1415/2-deer_psi[ind])+(deer_right-deer_ypos[ind])*cos(3.1415/2-deer_psi[ind]))]
                    # C: Passanger rear
                    deerC = [deer_xpos[ind] + ((deer_back-deer_xpos[ind])*cos(3.1415/2-deer_psi[ind])-(deer_right-deer_ypos[ind])*sin(3.1415/2-deer_psi[ind])), deer_ypos[ind] + ((deer_back-deer_xpos[ind])*sin(3.1415/2-deer_psi[ind])+(deer_right-deer_ypos[ind])*cos(3.1415/2-deer_psi[ind]))]
                    # D: Driver rear
                    deerD = [deer_xpos[ind] + ((deer_back-deer_xpos[ind])*cos(3.1415/2-deer_psi[ind])-(deer_left-deer_ypos[ind])*sin(3.1415/2-deer_psi[ind])), deer_ypos[ind] + ((deer_back-deer_xpos[ind])*sin(3.1415/2-deer_psi[ind])+(deer_left-deer_ypos[ind])*cos(3.1415/2-deer_psi[ind]))]

                    deer = [deerA,deerB,deerC,deerD]

                    deerA = self.rotate(deerA[0],deerA[1],car_xpos[ind],car_ypos[ind],-car_psi[ind])
                    deerB = self.rotate(deerB[0],deerB[1],car_xpos[ind],car_ypos[ind],-car_psi[ind])
                    deerC = self.rotate(deerC[0],deerC[1],car_xpos[ind],car_ypos[ind],-car_psi[ind])
                    deerD = self.rotate(deerD[0],deerD[1],car_xpos[ind],car_ypos[ind],-car_psi[ind])

                    deerA[0] = deerA[0]-car_xpos[ind]
                    deerA[1] = deerA[1]-car_ypos[ind]
                    deerB[0] = deerB[0]-car_xpos[ind]
                    deerB[1] = deerB[1]-car_ypos[ind]
                    deerC[0] = deerC[0]-car_xpos[ind]
                    deerC[1] = deerC[1]-car_ypos[ind]
                    deerD[0] = deerD[0]-car_xpos[ind]
                    deerD[1] = deerD[1]-car_ypos[ind]


                    deerA = self.checkBounds(deerA[0],deerA[1],x_max=self.car_b,x_min=-self.car_a,y_max=0.5*self.car_width,y_min=-0.5*self.car_width)
                    deerB = self.checkBounds(deerB[0],deerB[1],x_max=self.car_b,x_min=-self.car_a,y_max=0.5*self.car_width,y_min=-0.5*self.car_width)
                    deerC = self.checkBounds(deerC[0],deerC[1],x_max=self.car_b,x_min=-self.car_a,y_max=0.5*self.car_width,y_min=-0.5*self.car_width)
                    deerD = self.checkBounds(deerD[0],deerD[1],x_max=self.car_b,x_min=-self.car_a,y_max=0.5*self.car_width,y_min=-0.5*self.car_width)

                    if (deerA==True or deerB==True or deerC==True or deerD==True):
                        Collision = True
                    else:
                        pass


                else:
                    pass

        else:
            pass
            
        return Collision




def Demo_CV():


    Collision = CollisionCheck()
    dirname = '../Figure_Scripts/CV_Deer/MPC_F_braking_KinCar_10_0_1_1_0_CV_sight_80_swerve_80/'

    for speed, filename_speed in enumerate(sorted(os.listdir(dirname))):
        dirname_speed = dirname + str(filename_speed)
        speed_str = filename_speed[-2:]
        for angle, filename_angle in enumerate(sorted(os.listdir(dirname_speed))):
            dirname_angle = dirname_speed + '/' + str(filename_angle)
            angle_str = filename_angle[-3:]
            for file, filename in enumerate(sorted(os.listdir(dirname_angle))):
                if file == 1:
                    data = loadtxt(dirname_angle+'/'+filename)
                    car_xpos = data[:,5]
                    car_ypos = data[:,3]
                    car_psi = data[:,7]
                    deer_xpos = data[:,17]
                    deer_ypos = data[:,18]
                    deer_psi = data[:,16] 
                    coll = Collision.collision(car_xpos,car_ypos,car_psi,deer_xpos,deer_ypos,deer_psi)
                    if coll == True:
                        print speed_str ,angle_str

if __name__=='__main__':
    Demo_CV()


    # car = [[37.899337827630575, 1.6664977765391549], [38.032538474867472, -0.32906169077212322], [34.141197513610479, -0.58880295288406803], [34.007996866373588, 1.4067565144272101]]
    # deer = [[37.045925915033408, 0.0027476343679303794], [37.072139385295088, 0.5020600155250976], [38.570076528766592, 0.42341960474006957], [38.543863058504911, -0.075892776417097602]]
  
    # figure()
    # print car[0][1]
    # plot(car[0][0],car[0][1],'o')
    # plot(car[1][0],car[1][1],'o')
    # plot(car[2][0],car[2][1],'o')
    # plot(car[3][0],car[3][1],'o')
    # plot(deer[0][0],deer[0][1],'o')
    # plot(deer[1][0],deer[1][1],'o')
    # plot(deer[2][0],deer[2][1],'o')
    # plot(deer[3][0],deer[3][1],'o')
    # axis('equal')

    # show()

                    

