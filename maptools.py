""" This is a class for handling low-dimensional maps. Multiple functions exist for loading maps from databases, internet sources, csv files. Maps can be of varying format (?)
Once your map is loaded, in a form [X Y Z] (this will change as we add roll, number of features, feature offsets, etc.), you may request an S vector.
You can also request a vector of map xyz points for the road center given a vehicle pose array([x,y,z,r,p,y]) and a numpy array of relative S values you wish to see.

The module will be extended as we move to more complex map representations and map loading schemes, like databases, etc.

The demo shows you what the module can do using matplotlib. you will see a sample map (included with this module) and a vehicle pose, and relative map positions in front of the vehicle.
"""
#Alexander Brown
#December 29, 2013
#alexanderallenbrown@gmail.com
#Bobby Leary
#bobbyleary@gmail.com

import os
import csv
# This file provides all of the necessary conversions from GPS coordinates to local ENU map coordinates
from math import cos,sin, atan2
import math #just in case we need other stuff.
from numpy import *
from matplotlib.pyplot import *
from scipy.signal import medfilt



class Map():

	def __init__(self,type='csv',array = None,filename='map/lidar-map-smooth.csv',height_ref='WGS', ireflat = None, ireflon = None, irefelev = None):
		""" Initializes map class. You may choose map type csv, or... height_ref determines whether to subtract ellipsoid height (33.8m at test track). Currently does nothing, TODO!!"""
		cwd = os.getcwd()+'/maptools_folder/'
		#print cwd
		if type=='csv':
			self.loadCSVLLAMap(cwd+filename, ireflat, ireflon, irefelev)
		elif type == 'array':
			self.loadCSVLLAMap_array(array, ireflat, ireflon, irefelev)			

		elif type=='xyz':
			self.loadCSVXYZMap(cwd+filename)
		else:
			print "you did not specify a legitimate map type. Try Again"
			self.X = []
			self.Y = []
			self.Z = []
			self.S = []
			self.roadpitch=[]
			self.roadyaw=[]
			

	def loadCSVLLAMap_array(self,array,ireflat=None,ireflon=None,irefelev=None):
		""" This function loads the CSV map of lat,lon,elev points, and returns the array of xyz points"""
		map_reader = array
		firstline = 1 #set this flag to show we are at the first position
		#need an old position so we can calculate station, yaw and pitch angle of road. roll assumbed 0 for now.
		xgps_old = 0
		ygps_old = 0
		zgps_old = 0
		station=0
		for row in map_reader:
			mapline = row
			#if we have a real line...
			if len(mapline)>2:
				#now turn these into floats
				lat = float(mapline[0])
				lon = float(mapline[1])
				elev = float(mapline[2])
				#print lat,lon,elev
				if firstline==1:
					if ireflat==None:
						reflat = lat
						reflon = lon
						refalt = elev
					
					else:
						reflat = ireflat
						reflon = ireflon
						refalt = irefelev

					self.origin_x=reflat
					self.origin_y=reflon
					self.origin_z=refalt#-33.842#FIX THIS WHEN NMEA IS FIXED!!!    
						
				#now we will publish a marker for the current point. should automatically go to zero for the first point (origin)   
				xgps,ygps,zgps = self.wgslla2enu(lat,lon,elev,reflat,reflon,refalt)#-33.842
				#now the marker message is taken care of, and we need station, pitch, and yaw from this point and its predecessor
				roadyaw = atan2(ygps-ygps_old,xgps-xgps_old) #easy peasy.
				#roadyaw = atan2(xgps-xgps_old,ygps-ygps_old)
				delta_station = ((xgps-xgps_old)**2+(ygps-ygps_old)**2+(zgps-zgps_old)**2)**.5 #get the actual distance between the map points
				station=station+delta_station#get the new station value 
				roadpitch = atan2(zgps_old-zgps,delta_station)#pitch is positive when the road is DOWNHILL by ENU convention
				if firstline==1:
					self.X=xgps
					self.Y=ygps
					self.Z=zgps
					self.S = station
					self.roadpitch = roadpitch
					self.roadyaw = roadyaw
					firstline=0
				else:
					self.X = append(self.X,xgps)
					self.Y = append(self.Y,ygps)
					self.Z = append(self.Z,zgps)
					self.S = append(self.S,station)
					self.roadpitch = append(self.roadpitch,roadpitch)
					self.roadyaw = append(self.roadyaw,roadyaw)
				xgps_old=xgps
				ygps_old=ygps
				zgps_old=zgps
		self.roadpitch[0]=self.roadpitch[1]
		self.roadyaw[0] = self.roadyaw[1]
		self.calcCurvature()


	def loadCSVLLAMap(self,filename,ireflat=None,ireflon=None,irefelev=None):
		""" This function loads the CSV map of lat,lon,elev points, and returns the array of xyz points"""
		map_reader = csv.reader(open(filename,'rb'))
		firstline = 1 #set this flag to show we are at the first position
		#need an old position so we can calculate station, yaw and pitch angle of road. roll assumbed 0 for now.
		xgps_old = 0
		ygps_old = 0
		zgps_old = 0
		station=0
		for row in map_reader:
			mapline = row
			#if we have a real line...
			if len(mapline)>2:
				#now turn these into floats
				lat = float(mapline[0])
				lon = float(mapline[1])
				elev = float(mapline[2])
				#print lat,lon,elev
				if firstline==1:
					if ireflat==None:
						reflat = lat
						reflon = lon
						refalt = elev
					
					else:
						reflat = ireflat
						reflon = ireflon
						refalt = irefelev

					self.origin_x=reflat
					self.origin_y=reflon
					self.origin_z=refalt#-33.842#FIX THIS WHEN NMEA IS FIXED!!!    
						
				#now we will publish a marker for the current point. should automatically go to zero for the first point (origin)   
				xgps,ygps,zgps = self.wgslla2enu(lat,lon,elev,reflat,reflon,refalt)#-33.842
				#now the marker message is taken care of, and we need station, pitch, and yaw from this point and its predecessor
				roadyaw = atan2(ygps-ygps_old,xgps-xgps_old) #easy peasy.
				#roadyaw = atan2(xgps-xgps_old,ygps-ygps_old)
				delta_station = ((xgps-xgps_old)**2+(ygps-ygps_old)**2+(zgps-zgps_old)**2)**.5 #get the actual distance between the map points
				station=station+delta_station#get the new station value 
				roadpitch = atan2(zgps_old-zgps,delta_station)#pitch is positive when the road is DOWNHILL by ENU convention
				if firstline==1:
					self.X=xgps
					self.Y=ygps
					self.Z=zgps
					self.S = station
					self.roadpitch = roadpitch
					self.roadyaw = roadyaw
					firstline=0
				else:
					self.X = append(self.X,xgps)
					self.Y = append(self.Y,ygps)
					self.Z = append(self.Z,zgps)
					self.S = append(self.S,station)
					self.roadpitch = append(self.roadpitch,roadpitch)
					self.roadyaw = append(self.roadyaw,roadyaw)
				xgps_old=xgps
				ygps_old=ygps
				zgps_old=zgps
		self.roadpitch[0]=self.roadpitch[1]
		self.roadyaw[0] = self.roadyaw[1]
		self.calcCurvature()

	def calcCurvature(self):
		#now calculate the kurvature at every point in the track.
		# k ~ d^2y/dS^2
		Kthresh = 0.002#only recognize curves of 200m radius or smaller
		self.K = medfilt(append(array([0]), diff(self.roadyaw)/diff(self.S)),5);
		#then 1/K will be the turn radius
		self.Knz = self.K[abs(self.K)>Kthresh];
		self.Snz = self.S[abs(self.K)>Kthresh];
		self.Rnz = 1/self.Knz;


		#now, let's look at plotting the turn center on the track where the radius
		#is nonzero.
		self.roadyawnz = self.roadyaw[abs(self.K)>Kthresh];
		self.Xnz = self.X[abs(self.K)>Kthresh];
		self.Ynz = self.Y[abs(self.K)>Kthresh];
		#the turn center is R away, at roadyaw+pi/2 radians.
		self.Xc = self.Xnz+self.Rnz*cos(self.roadyawnz+pi/2);
		self.Yc = self.Ynz+self.Rnz*sin(self.roadyawnz+pi/2);
		self.nzind = where(abs(self.K)>Kthresh)#indices where there are turns



		
	def loadCSVXYZMap(self,filename):
		""" This function loads the CSV map of lat,lon,elev points, and returns the array of xyz points"""
		xgps_old,ygps_old,zgps_old=0,0,0
		map_reader = csv.reader(open(filename,'rb'))
		firstline = 1 #set this flag to show we are at the first position
		#need an old position so we can calculate station, yaw and pitch angle of road. roll assumbed 0 for now.
		station = 0
		for row in map_reader:
			
			mapline = row
			#if we have a real line...
			if len(mapline)>2:
				#now turn these into floats
				xgps = float(mapline[0])
				ygps = float(mapline[1])
				zgps = float(mapline[2])
				#print lat,lon,elev
				if firstline==1:
					self.origin_=xgps
					self.origin_y=ygps
					self.origin_z=zgps#-33.842#FIX THIS WHEN NMEA IS FIXED!!!    
					
				roadyaw = atan2(ygps-ygps_old,xgps-xgps_old) #easy peasy.
				#roadyaw = atan2(xgps-xgps_old,ygps-ygps_old)
				delta_station = ((xgps-xgps_old)**2+(ygps-ygps_old)**2+(zgps-zgps_old)**2)**.5 #get the actual distance between the map points
				station=station+delta_station#get the new station value 
				print delta_station
				roadpitch = atan2(zgps_old-zgps,delta_station)#pitch is positive when the road is DOWNHILL by ENU convention
				if firstline==1:
					self.X=xgps
					self.Y=ygps
					self.Z=zgps
					self.S = station
					self.roadpitch = roadpitch
					self.roadyaw = roadyaw
					firstline=0
				else:
					self.X = append(self.X,xgps)
					self.Y = append(self.Y,ygps)
					self.Z = append(self.Z,zgps)
					self.S = append(self.S,station)
					self.roadpitch = append(self.roadpitch,roadpitch)
					self.roadyaw = append(self.roadyaw,roadyaw)
				xgps_old=xgps
				ygps_old=ygps
				zgps_old=zgps
		self.roadpitch[0]=self.roadpitch[1]
		self.roadyaw[0] = self.roadyaw[1]

	def rot(self,angle, axis):
		""" General rotation function for RPY-style rotations"""
		#function R=rot(angle (degrees), axis) 
		pi = 3.141592654
		cang=cos(angle*pi/180);
		sang=sin(angle*pi/180);
		R = eye(3)
		
		if (axis==1):
			R[1,1]=cang;
			R[2,2]=cang;
			R[1,2]=sang;
			R[2,1]=-sang;
		

		if (axis==2):
			R[0,0]=cang;
			R[2,2]=cang;
			R[0,2]=-sang;
			R[2,0]=sang;
		

		if (axis==3):
			R[0,0]=cang;
			R[1,1]=cang;
			R[1,0]=-sang;
			R[0,1]=sang;
		
		return R


	def wgslla2enu(self,lat, lon, alt, reflat, reflon, refalt):
		"""Goes directly from LLA to ENU coordinates at a given reference location"""
		(x,y,z) = self.wgslla2xyz(lat, lon, alt);
		east,north,up = self.WGSxyz2ENU(x,y,z, reflat, reflon, refalt);
		return east,north,up
		

	 
	def wgslla2xyz(self,wlat, wlon, walt):
		""" Goes from LLA to ECEF coordinates"""
		pi = 3.141592654
		A_EARTH = 6378137;
		flattening = 1/298.257223563;
		NAV_E2 = (2-flattening)*flattening; # also e^2
		deg2rad = pi/180;

		slat = sin(wlat*deg2rad);
		clat = cos(wlat*deg2rad);
		r_n = A_EARTH/sqrt(1 - NAV_E2*slat*slat);
		x=(r_n+walt)*clat*cos(wlon*deg2rad);  
		y=(r_n+walt)*clat*sin(wlon*deg2rad);
		z=((r_n*(1-NAV_E2)+walt)*slat);
		return x,y,z


	def WGSxyz2ENU(self,x,y,z, reflat, reflon, refalt):
		""" goes from ECEF coordinates to ENU coordinates at a reference location"""
		# First, calculate the xyz of reflat, reflon, refalt
		(refx,refy,refz) = self.wgslla2xyz(reflat, reflon, refalt);
		# Difference xyz from reference point
		
		xyz = array([[x],[y],[z]])
		refxyz = array([[refx],[refy],[refz]])
		diffxyz = xyz - refxyz;

		# Now rotate the (often short) diffxyz vector to enu frame

		R1=self.rot(90+reflon, 3);
		R2=self.rot(90-reflat, 1);
		R=dot(R2,R1);

		enu=dot(R,diffxyz);
		
		east = float(enu[0])
		north = float(enu[1])
		up = float(enu[2])
		return east,north,up

	def enu2wgsxyz(self,east, north, up, reflat, reflon, refalt):
		#First, rotate the enu vector to xyz frame
		R1=self.rot(90+reflon, 3)
		R2=self.rot(90-reflat, 1)
		R=dot(R2,R1)
		
		enu = array([[east],[north],[up]])
		diffxyz=dot(linalg.inv(R), enu)
		
		#Then, calculate the xyz of reflat, reflon, refalt
		x_ref, y_ref, z_ref = self.wgslla2xyz(reflat, reflon, refalt);
		
		# Add diffxyz to refxyz
		x = x_ref + diffxyz[0]
		y = y_ref + diffxyz[1] 
		z = z_ref + diffxyz[2] 
		
		return x, y, z

	def wgsxyz2lla(self,x, y, z):

		#This dual-variable iteration seems to be 7 or 8 times faster than
		#  a one-variable (in latitude only) iteration.  AKB 7/17/95 

		A_EARTH = 6378137;
		flattening = 1/298.257223563;
		NAV_E2 = (2-flattening)*flattening; # also e^2
		rad2deg = 180/pi;

		if ((x == 0.0) and (y == 0.0)):
			wlon = 0.0;
		else:
			wlon = atan2(y, x)*rad2deg;

		#Make initial lat and alt guesses based on spherical earth.
		rhosqrd = x*x + y*y;
		rho = sqrt(rhosqrd);
		templat = atan2(z, rho);
		tempalt = sqrt(rhosqrd + z*z) - A_EARTH;
		rhoerror = 1000.0;
		zerror   = 1000.0;
		#Newton's method iteration on templat and tempalt makes
		#the residuals on rho and z progressively smaller.  Loop
		#is implemented as a 'while' instead of a 'do' to simplify
		#porting to MATLAB

		while ((absolute(rhoerror) > 1e-6) or (absolute(zerror) > 1e-6)):
			slat = sin(templat);
			clat = cos(templat);
			q = 1 - NAV_E2*slat*slat;
			r_n = A_EARTH/sqrt(q);
			drdl = r_n*NAV_E2*slat*clat/q; # d(r_n)/d(latitutde) 

			rhoerror = (r_n + tempalt)*clat - rho;
			zerror   = (r_n*(1 - NAV_E2) + tempalt)*slat - z;

			#             --                               -- --      --
			#             |  drhoerror/dlat  drhoerror/dalt | |  a  b  |
			# Find Jacobian           |                     |=|        |
			#             |   dzerror/dlat    dzerror/dalt  | |  c  d  |
			#             --                               -- --      -- 

			aa = drdl*clat - (r_n + tempalt)*slat;
			bb = clat;
			cc = (1 - NAV_E2)*(drdl*slat + r_n*clat);
			dd = slat;

			#  Apply correction = inv(Jacobian)*errorvector
			invdet = 1.0/(aa*dd - bb*cc);
			templat = templat - invdet*(+dd*rhoerror -bb*zerror);
			tempalt = tempalt - invdet*(-cc*rhoerror +aa*zerror);

		wlat = templat*rad2deg;
		walt = tempalt;
		return wlat, wlon, walt

	def localyoffset_to_mapframe(self,S,offset,roll=0):
		''' this function takes an S-location in the map, and uses the GPS functions and a simple (TODO) planar coordinate transform to transform this
		into map XYZ . it returns  x,y,z as separate numbers.'''
		#first get the coords of the lane center
		Xmap_here = interp(S,self.S,self.X)
		Ymap_here = interp(S,self.S,self.Y)
		Zmap_here = interp(S,self.S,self.Z)
		psimap_here = interp(S,self.S,self.roadyaw)
		#then get the  global coords of the requested point. right now we don't deal with road's roll... TODO TODO!! 
		X_here = Xmap_here+offset*cos(psimap_here-pi/2)
		Y_here = (Ymap_here+offset*sin(psimap_here-pi/2))*cos(roll)
		Z_here = Zmap_here+offset*sin(roll) #This should be changed eventually TODO!!!

		return X_here,Y_here,Z_here

	def station_here(self,latx,lony,altz,type="lla"):
		""" this function takes a latitude and a longitude, and finds your S location on the map through orthogonal projection. Also returns current path error.
		If you wish to input x and y and z coordinates instead of lat, lon, altitude, pass the fourth argument in station_here(latx,lony,altz,type) as 'xyz' """
		#here, D is our set of n-dimensional vectors (map points) and x is our query vector. K is the number of neighbors we want.
		#first, make a 2xn array of the mapped x and y positions.
		xymap = np.vstack([self.X,self.Y])
		#then, convert our LLA input to an x,y IF necessary
		if type=="lla":
			#then convert to xyz
			x,y,z = self.wgslla2enu(latx,lony,altz,self.origin_x,self.origin_y,self.origin_z)
			gpspos = np.vstack([x,y])
		else:
			if type is not "xyz":
				print "Hey! What are you doing? You haven't entered type lla or type xyz. Trying type xyz:"
			x = latx
			y = lony
			gpspos = np.vstack([x,y])
	
		twoneighbors = self.knn_search(gpspos,xymap,2)
		#now we have the two indices of the closest points. we need now to find S!
		lmapS = self.S[twoneighbors[0]]
		lmapx = self.X[twoneighbors[0]]
		lmapy = self.Y[twoneighbors[0]]
		psimap = self.roadyaw[twoneighbors[0]]
		#find the global X and Y difference between our query point and the nearest point on the map
		delX = -lmapx+gpspos[0][0]
		delY = -lmapy+gpspos[1][0]
		#now use the nearest point and the local yaw tangent angle of the map to find our current S (piecewise linear approximation) and offset 
		S=delX*cos(-psimap)-delY*sin(-psimap)+self.S[twoneighbors[0]]
		y_err=delX*sin(-psimap)+delY*cos(-psimap)#local yhat (like the one that comes out of preview filter).
		return S,y_err,psimap

	def knn_search(self,x, D, K):
		""" find K nearest neighbours of data among D """
		ndata = D.shape[1]
		K = K if K < ndata else ndata
		# euclidean distances from the other points
		sqd = np.sqrt(((D - x[:,:ndata])**2).sum(axis=0))
		idx = np.argsort(sqd) # sorting
		# return the indexes of K nearest neighbours
		return idx[:K]  


	def relativeMapPoints(self,x=0,y=0,z=0,roll=0,pitch=0,yaw=0,Svector=arange(0,10)):
		"""This function takes a Pose as X,Y,Z,roll,pitch,yaw (angles in radians) and a vector of S values in front of the vehicle (relative to the vehicle) at which to find local map coordinates through interpolation
		angles for rotation are to be used for ISO coordinates. x is positive forward, y is positive left, z is positive up referenced to the driver.
		"""
		S,offset,roadyaw = self.station_here(x,y,z,'xyz') #find the current S coordinate, and some extra stuff we don't need
		Squery = Svector+S # this allows us to ask for points in the map as a 1:1 lookup, using interpolation
		Xpts = interp(Squery,self.S,self.X)
		Ypts = interp(Squery,self.S,self.Y)
		Zpts = interp(Squery,self.S,self.Z)
		
		xpts, ypts, zpts = self.mapFrameToLocalFrame(Xpts,Ypts,Zpts,x,y,z,roll,pitch,yaw)

		return xpts, ypts, zpts

	def drawMeOnMap(self,mylat=0,mylon=0,myalt=0,roll=0,pitch=0,yaw=0,type='lla'):
		""" This function uses matplotlib to draw you on a map.
		drawMeOnMap(mylat=0,mylon=0,myalt=0,roll=0,pitch=0,yaw=0)
		latitude, longitude, (deg) altitude (m), roll,pitch,yaw (rad)
		"""
		#plot the plan view of the road center map
		figure()
		plot(self.X,self.Y,'k')
		#Also in this figure, let's take the opportunity to test out the Station calculator (KNN-based)
		#let's see how accurate this is! Let's plot this point, with a radius of "offset" and an annotation saying what we calculated as the station!
                if type=='lla':
                        S,offset,psimap = self.station_here(mylat,mylon,myalt,'lla') #should return the effective station value and the offset from the path.
                        x,y,z = self.wgslla2enu(mylat,mylon,myalt,self.origin_x,self.origin_y,self.origin_z)
                elif type=='xyz':
                        S,offset,psimap = self.station_here(mylat,mylon,myalt,'xyz') #should return the effective station value and the offset from the path.
                        x,y,z = mylat,mylon,myalt
		l = 2 #length for plotting a line representing the road tangent
		arrow(x,y,l*cos(psimap),l*sin(psimap),color='green')
		#now plot a line with length "offset" pointing back towards the map to test the offset capability
		arrow(x,y,offset*cos(psimap-pi/2),offset*sin(psimap-pi/2),color='red')
		annotate('S='+str(S)+" offset= "+str(offset),xy=(x,y),xytext=(x+1,y+1))

		xlabel('Global X (m)')
		ylabel('Global Y (m)')
		title('Map Plan View, Vehicle, Offset')
		axis('equal') 
		show()   
	def drawLocalPts(self,Svec,xpts,ypts,zpts):
		figure()
		subplot(3,1,1)
		plot(Svec,xpts,'k')
		title('Map Points for a range ahead in local coordinates')
		ylabel('local x')
		subplot(3,1,2)
		plot(Svec,ypts,'k')
		ylabel('local y')
		subplot(3,1,3)
		plot(Svec,zpts,'k')
		ylabel('local z')
		xlabel('Station ahead')  
		show()  
	

	def mapFrameToLocalFrame(self,xvecglobal,yvecglobal,zvecglobal,x=0,y=0,z=0,roll=0,pitch=0,yaw=0):
		""" mapFrameToLocalFrame(self,type='lla',lat=None,lon=None,elev=None,x=None,y=None,z=None,roll=0,pitch=0,yaw=0,xvecgobal,yvecglobal,zvecglobal):
			returns: xveclocal,yveclocal,zveclocal

			The rotation order is zyx (yaw pitch roll). angles are in radians.
			The size and order of xvecglobal,yvecglobal,zvecglobal is UNCHANGED.
		"""

		# Create the translation vector
		T = array([[1,0,0,-x],[0,1,0,-y],[0,0,1,-z],[0,0,0,1]])

		# Create the rotation matrices for the X, Y, and Z rotations
		# We are rotating a point, so the negative sines are in the right location
		Rx = array([[1,0,0,0],[0,cos(roll),sin(roll),0],[0,-sin(roll),cos(roll),0],[0,0,0,1]])
		Ry = array([[cos(pitch),0,-sin(pitch),0],[0,1,0,0],[sin(pitch),0,cos(pitch),0],[0,0,0,1]])
		Rz = array([[cos(yaw),sin(yaw),0,0],[-sin(yaw),cos(yaw),0,0],[0,0,1,0],[0,0,0,1]])

		# Create our rotation matrix (Rx*Ry*Rz)
		R = dot(dot(Rx,Ry),Rz)

		xveclocal,yveclocal,zveclocal=array([]),array([]),array([])
		P=dot(R,T)
		#now transform each point
		if len(xvecglobal)==len(yvecglobal) and len(yvecglobal)==len(zvecglobal):
			for ind in range(0,len(xvecglobal)):
				#transform this point and add to vector
				local=dot(P,vstack([xvecglobal[ind],yvecglobal[ind],zvecglobal[ind],1]))
				xveclocal=append(xveclocal,local[0])
				yveclocal=append(yveclocal,local[1])
				zveclocal=append(zveclocal,local[2])

		return xveclocal, yveclocal, zveclocal

	def pathCoordinatesToLocalCoordinates(self,Svec,yvec,x,y,z,roll,pitch,yaw):
		""" pathCoordinatesToLocalCoordinates(self,Svec,yvec,x,y,z,roll,pitch,yaw)
			returns: xlocal, ylocal, zlocal (Vectors of points in car referenced frame)

		"""
		#TRANSFORM POINTS INTO ENU FRAM
		xmap,ymap,zmap=array([]),array([]),array([])
		for ind in range(0,len(Svec)):
			currxmap,currymap,currzmap = self.localyoffset_to_mapframe(Svec[ind],yvec[ind])
			xmap=append(xmap,currxmap)
			ymap=append(ymap,currymap)
			zmap=append(zmap,currzmap)

		#TRANSFORM TO LOCAL FRAME
		return self.mapFrameToLocalFrame(xmap,ymap,zmap,x,y,z,roll,pitch,yaw)

if __name__ == '__main__':
	""" this is a demo for the maptools module. It shows how to initialize a map, and what is available in the class, using plots. """
	print "You have chosen to run the demo."
	#first, we initialize a map instance. It will use the default filename in this case, and load the map.
	mymap = Map()
	
	#let's get an arbitrary pose as you might get from a GPS/IMU going. this will be used to test some of the other features of this module (local map point calculation)
	###### This portion deals with just loading the map and testing the various map properties #########
	mylat=40.86497201062075
	mylon=-77.8364413039564
	myalt=336.6970515018253

	#plot the plan view of the road center map
	f1 = figure()
	plot(mymap.X,mymap.Y,'k')
	xlabel('Global X (m)')
	ylabel('Global Y (m)')
	title('Map Plan View')
	axis('equal')
	#this next piece will show how to visualize the station coordinate for the given map.
	uniformS = arange(0,max(mymap.S),100) # uniform values of S in 10m increments.
	for ind in range(0,len(uniformS)): #for each of these values,
		X = interp(uniformS[ind],mymap.S,mymap.X) #what is the X value of the map at the S I've given?
		Y = interp(uniformS[ind],mymap.S,mymap.Y) #what is the Y value of the map at the S I've given?
		annotate('S='+str(uniformS[ind]),xy=(X,Y))

	#Also in this figure, let's take the opportunity to test out the Station calculator (KNN-based)
	S,offset,psimap = mymap.station_here(mylat,mylon,myalt,'lla') #should return the effective station value and the offset from the path.
	#let's see how accurate this is! Let's plot this point, with a radius of "offset" and an annotation saying what we calculated as the station!
	x,y,z = mymap.wgslla2enu(mylat,mylon,myalt,mymap.origin_x,mymap.origin_y,mymap.origin_z)
	l = 10 #length for plotting a line representing the road tangent
	arrow(x,y,l*cos(psimap),l*sin(psimap),color='green')
	#now plot a line with length "offset" pointing back towards the map to test the offset capability
	arrow(x,y,offset*cos(psimap-pi/2),offset*sin(psimap-pi/2),color='red')
	annotate('S='+str(S)+" offset= "+str(offset),xy=(x,y),xytext=(x+1,y+1))

	mymap.drawMeOnMap(mylat,mylon,myalt,0,0,psimap)

	#now we will plot the road pitch and road yaw vs. station to show how that works
	f2 = figure()
	subplot(2,1,1)
	plot(mymap.S,mymap.roadpitch*180/pi,'k')
	title('Road Pitch and Yaw Angles as a Function of Station')
	ylabel('Road pitch angle (deg)')

	subplot(2,1,2)
	plot(mymap.S,mymap.roadyaw*180/pi,'k')
	ylabel('Road yaw angle (deg)')
	xlabel('Station (m)')

	figure()
	subplot(3,1,1)
	plot(mymap.S,mymap.X,'k')
	ylabel('X coordinate (m)')
	title('X Y and Z vs. S')
	subplot(3,1,2)
	plot(mymap.S,mymap.Y,'k')
	ylabel('Y Coordinate (m)')
	subplot(3,1,3)
	plot(mymap.S,mymap.Z,'k')
	ylabel('Z Coordinate (m)')
	xlabel('Station (m)')


	############ This portion deals with how you can transform a vector of map points into an arbitrary coordinate space! ##########
	myroll = 0
	mypitch = 0
	myyaw = psimap+5*pi/180 #arbitrarily add yaw angle to whatever the map yaw was.

	Svec = arange(0,20,.1)
	#now we try out the local map stuff. I chose a downward tilt angle of 10 degrees, and a little yaw offset too.
	xpts,ypts,zpts = mymap.relativeMapPoints(x,y,0,0,10.0*pi/180,myyaw,Svec)
	figure()
	subplot(3,1,1)
	plot(Svec,xpts,'k')
	title('Map Points for a range ahead in local coordinates')
	ylabel('local x')
	subplot(3,1,2)
	plot(Svec,ypts,'k')
	ylabel('local y')
	subplot(3,1,3)
	plot(Svec,zpts,'k')
	ylabel('local z')
	xlabel('Station ahead')

	figure()
	plot(xpts,ypts,'k')
	xlabel('local x (m)')
	ylabel('local y (m)')

	########### This section tests the mapFrameToLocalFrame function by converting the map coordinates to the self referenced frame

	S,offset,roadyaw = mymap.station_here(x,y,z,'xyz')
	Squery = Svec+S
	Xpts = interp(Squery,mymap.S,mymap.X)
	Ypts = interp(Squery,mymap.S,mymap.Y)
	Zpts = interp(Squery,mymap.S,mymap.Z)

	xpts_, ypts_, zpts_ = mymap.mapFrameToLocalFrame(Xpts,Ypts,Zpts,x=x,y=y,z=0,roll=0,pitch=10.0*pi/180,yaw=myyaw)

	figure()
	subplot(3,1,1)
	plot(Svec,xpts,'k',Svec,xpts_,'r')
	title('Map Points for a range ahead in local coordinates')
	ylabel('local x')
	subplot(3,1,2)
	plot(Svec,ypts,'k',Svec,ypts_,'r')
	ylabel('local y')
	subplot(3,1,3)
	plot(Svec,zpts,'k',Svec,zpts_,'r')
	ylabel('local z')
	xlabel('Station ahead')

	########### This section tests the pathCoordinatesToLocalCoordinates by passing in the select S coordinates and local yoffsets of an example feature (lane marker)

	yoffsets = -1.625*ones((len(Svec)))

	xveclocal, yveclocal, zveclocal = mymap.pathCoordinatesToLocalCoordinates(Squery,yoffsets,x=x,y=y,z=0,roll=0,pitch=10.0*pi/180,yaw=myyaw)

	figure()
	plot(xveclocal,yveclocal,'g',xpts,ypts,'k',xpts_,ypts_,'r')
	xlabel('local x (m)')
	ylabel('local y (m)')

	show()
