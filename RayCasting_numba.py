import sys
from numpy import *
from matplotlib.pyplot import *
from KMLtoXYZ import *
from numba import jit,float32,int32

@jit(float32[:](float32[:], float32[:]))
def distanceray(pos, array):
    
    # Points of array (Begin and end with the same point)

    #print 'array', array
    P = double(array)

    dist_angle = zeros(360)

    for angle_deg in range(0,360):

        angle = angle_deg/180.0*3.1416

        dist_vec = zeros(len(P[0])-1)

        for seg in range(1,len(P[0])):

            # If segment is perpendicular:
            if (P[0,seg]==P[0,seg-1]):

                xint = P[0,seg]

                yint = tan(angle)*(xint-pos[0])+pos[1]

                #print seg    

            # If segment is not perpendicular:
            else:

                dydx_seg = ((P[1,seg]-P[1,seg-1])/(P[0,seg]-P[0,seg-1]))

                xint = (dydx_seg*P[0,seg-1]-P[1,seg-1]-tan(angle)*pos[0]+pos[1])/(dydx_seg-tan(angle))

                yint = (P[1,seg]-P[1,seg-1])/(P[0,seg]-P[0,seg-1])*(xint-P[0,seg-1])+P[1,seg-1]

            #print xint, yint

            # Check that the ray intercepts the segment
            if ((((xint >= P[0,seg-1]) and (xint <= P[0,seg])) or ((xint >= P[0,seg]) and (xint <= P[0,seg-1]))) and (((yint >= P[1,seg-1]) and (yint <= P[1,seg])) or ((yint >= P[1,seg]) and (yint <= P[1,seg-1])))):
                dist_vec[seg-1] = sqrt((yint-pos[1])**2+(xint-pos[0])**2)

            else:

                #print 'Out of segment Range'

                dist_vec[seg-1] = NaN

            # Check that the interception is for positive ray
            if (angle_deg>=90 and angle_deg<270):
                if (xint>pos[0]):

                    #print 'Negative Ray'
                    dist_vec[seg-1] = NaN

            else:
                if (xint<pos[0]):
                    dist_vec[seg-1] = NaN


            #print dist_vec

        dist_vec = [100000 if isnan(x) else x for x in dist_vec]
        dist_angle[angle_deg] = nanmin(dist_vec)



        if (dist_angle[angle_deg]>100):
            dist_angle[angle_deg] = 100

    #print dist_angle

    angle_plot = linspace(0,360,360)

    # figure()
    # plot(angle_plot,dist_angle)
    # title('Ray Casting')
    # xlabel('Angle (deg)')
    # ylabel('Minimum Distance (m)')

    # show()

    return dist_angle

def ProbDist(pos, psi1, mapa, KML = True, fake = 'nothing',q_dist = 1.0,q_dangle = 1/1000.0):

    distAngle=MapRaycasting(pos, mapa, KML, fake)

    distAngle_inv = 1/distAngle

    distAngle_inv = 1000.0*distAngle_inv - min(distAngle_inv)

    Prob_map = (distAngle_inv)/sum(distAngle_inv)

    deg = linspace(0,359,360)
    sd = 15.

    prob_escape1 = 1/(sqrt(2*3.1416*sd**2))*exp(-(deg-45)**2/(2*sd**2))
    prob_escape2 = 1/(sqrt(2*3.1416*sd**2))*exp(-(deg-315)**2/(2*sd**2))
    probtot_escape = prob_escape1+prob_escape2
    probtot_escape = probtot_escape/sum(probtot_escape)

    #print sum(Prob_map)
    #print sum(probtot_escape)
    Prob1 = 3*Prob_map+probtot_escape
    Prob1 = Prob1/sum(Prob1)
    #print sum(Prob1)



    # deltaPsi = zeros(360)

    # for k in range(0,359):
    #     deltaPsi[k]=abs(k-psi1)

    #     if deltaPsi[k]>180:
    #         deltaPsi[k] = 360-deltaPsi[k]

    # Prob = q_dangle*(180-deltaPsi)*q_dist*distAngle

    # Prob1 = Prob/sum(Prob)

    angle_plot = linspace(0,360,360)


    # figure()
    # plot(angle_plot,Prob1)
    # plot(angle_plot,Prob_map)
    # plot(angle_plot,probtot_escape)
    # legend(['Final Probability Distribution','Attraction Probability Distribution','Repulsion Probability Distribution'])
    # title('Probability Distribution')
    # xlabel('Angle (deg)')
    # ylabel('Probability')  

    #show() 

    return Prob1

@jit(float32[:](float32[:],int32,int32,int32))
def MapRaycasting(pos, mapa, KML, fake):

    # print KML

    sight_dist = 60.0

    f = mapa

    if KML == 0:
        if fake == 0:#'nothing':
            Trees = [(array([-1000,1000,1000,-1000]),array([1000,1000,-1000,-1000]),array([0,0,0,0]))]
        if fake == 1:#'single_tree_60':
            Trees = [(array([58,62,62,58,58]),array([12,12,8,8,12]),array([0,0,0,0,0]))]
        if fake == 2:#'real_tree_wismer':
            Trees = [(array([100,180,180,100,100]),array([-4,-4,-94,-94,-4]),array([0,0,0,0,0]))]
    else:
        Trees = KMLtoXYZ(f)
    
    # print len(Trees)

    MinDist = []

    for ind in range(0,len(Trees)):
        MinDist.append(distanceray(pos, Trees[ind]))

    MinDist_Vec = zeros(len(MinDist[0]))

    d = zeros(len(MinDist))

    for ind in range(0,len(MinDist[0])):
        for ind1 in range(0,len(MinDist)):
            d[ind1] = MinDist[ind1][ind]

        MinDist_Vec[ind] = nanmin(d) 

        if isnan(MinDist_Vec[ind]):
            MinDist_Vec[ind] = sight_dist
        if MinDist_Vec[ind]>sight_dist:
            MinDist_Vec[ind] = sight_dist

    #print MinDist_Vec


    angle_plot = linspace(0,360,360)
    #hello
    # figure()
    # plot(angle_plot,MinDist_Vec)
    # title('Ray Casting')
    # xlabel('Angle (deg)')
    # ylabel('Minimum Distance (m)')

    # figure(facecolor='w')
    # subplot(111,facecolor='w')
    # seg = zeros((2,2))
    # print seg
    # for ind in range(0,360):
    #     seg[0][0] = pos[0]
    #     seg[0][1] = pos[0]+MinDist_Vec[ind]*cos(ind*3.1416/180)
    #     seg[1][0] = pos[1]
    #     seg[1][1] = pos[1]+MinDist_Vec[ind]*sin(ind*3.1416/180)
    #     plot(seg[0][:],seg[1][:],'k')
    # for ind in range(0,len(Trees)):
    #     plot(Trees[ind][0],Trees[ind][1],'g',linewidth = 2)
    #     fill(Trees[ind][0],Trees[ind][1],'g') 
    # plot(pos[0],pos[1],'ro')
    # axis('equal')
    # xlabel('X (m)', color='k')
    # ylabel('Y (m)', color='k')

    # show()

    return MinDist_Vec



if __name__=='__main__':

    #min_dist = MapRaycasting([200.0,0.0])

    dist = ProbDist([200.0,0.0],0.0, 'Test2.kml', KML=True, fake = 'real_tree_wismer')
    dist = MapRaycasting([62,0],'mapa',KML = False, fake = 'real_tree_wismer')

    angles = linspace(0,360,360)

    figure()
    plot(angles,dist)

    angle = zeros(10000)

    # for k in range(0,10000):
    #     angle[k] = random.choice(len(dist),1,p =dist)

    figure()
    hist(angle, bins = 360)

    show()

    print dist



