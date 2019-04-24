import pykml.parser as p
import csv
from numpy import *
from maptools import *
from matplotlib.pyplot import *


## RoadStart, RoadEnd, Elev, segments
def KMLtoXYZ(file):

    f = open(file)

    mymap = p.parse(f).getroot()

    #print dir(mymap)

    #print dir(mymap.Document.Folder.Placemark.LineString.coor)

    segments = 0

    ## Calculate the number of segments:
    for e in mymap.Document.Folder.Placemark:
    # Iterate through the number of line strings to caluclate the number of segments
        if 'LineString' in dir(e):
            segments= segments+1

    # print 'There are', segments, 'trees in this map.'

    Trees = []

    ind = 0

    for e in mymap.Document.Folder.Placemark:

        #print e.name

        if 'Point' in dir(e):
            strname = str(e.name)

            if strname == 'RoadStart':
                RoadStart = str(e.Point.coordinates)
                # print RoadStart

            if strname == 'RoadEnd':
                RoadEnd = str(e.Point.coordinates)

            if strname.find("Elev")>=0:
                Elevation = float(strname[4:])

                #print Elevation
        
        elif 'LineString' in dir(e):
            coor = str(e.LineString.coordinates)
            coor = str(coor)

            coor = coor.split(' ')

            for k in range(0,len(coor)):
                coor1 = coor[k]
                coor1 = coor1.split(',')
                coor[k] = coor1

            coor_start = coor[0][0]

            coor[0][0] = coor_start[6:]

            coor = coor[0:len(coor)-1]

            Trees.append(coor)

            ind= ind+1


    # Separate into Lat, Long, Elev
    RoadStart = RoadStart.split(',')
    RoadEnd = RoadEnd.split(',')

    # Add Elevation
    RoadStart[2] = str(Elevation)
    RoadEnd[2] = str(Elevation)

    # Switch Lat and Lon
    Lat_Start = RoadStart[1]
    Lon_Start = RoadStart[0]
    RoadStart[0] = Lat_Start
    RoadStart[1] = Lon_Start

    Lat_End = RoadEnd[1]
    Lon_End = RoadEnd[0]
    RoadEnd[0] = Lat_End
    RoadEnd[1] = Lon_End

    # Combine Start and End into one
    Road = vstack((RoadStart,RoadEnd))

    # Convert the Road from LLA to XYZ
    mapRoad = Map(type = 'array', array = Road,ireflat = float(RoadStart[0]), ireflon = float(RoadStart[1]), irefelev = float(RoadStart[2]))

    # Calculate the angle of rotation
    psi = (atan2(mapRoad.Y[1]-mapRoad.Y[0],mapRoad.X[1]-mapRoad.X[0]))

    # Rotate the road
    Road = mapRoad.mapFrameToLocalFrame(mapRoad.X,mapRoad.Y,mapRoad.Z,yaw = psi)

    # Add elevation to all Trees, and switch lat and lon
    for ind_1 in range(0,len(Trees)):
        for ind_2 in range(0,len(Trees[ind_1])):
            Trees[ind_1][ind_2][2] = str(Elevation)
            Trees_Lat = Trees[ind_1][ind_2][1]
            Trees_Lon = Trees[ind_1][ind_2][0]
            Trees[ind_1][ind_2][0] = Trees_Lat
            Trees[ind_1][ind_2][1] = Trees_Lon

    # Convert Trees from LLA to XYZ
    TreesXYZ = []
    for ind in range(0,segments):
        # Convert form LLA to XYZ
        mapTrees = Map(type = 'array', array = Trees[ind],ireflat = float(RoadStart[0]), ireflon = float(RoadStart[1]), irefelev = float(RoadStart[2]))
        # Rotate Trees
        XYZ = mapTrees.mapFrameToLocalFrame(mapTrees.X,mapTrees.Y,mapTrees.Z,yaw = psi)
        # Append to list
        TreesXYZ.append(XYZ)

     #Plot
    # figure()
    # plot(Road[0],Road[1])
    # for ind in range(0,segments):
    #     plot(TreesXYZ[ind][0],TreesXYZ[ind][1])
    # axis('equal')

    #show()

    return TreesXYZ

if __name__=='__main__':

    KMLtoXYZ('Test2.kml')
