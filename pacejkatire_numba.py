
from numba import jit,jitclass
from numba import float32 as nfloat32
from matplotlib.pyplot import *
import time


spec = [
    ('fyparms', nfloat32[:]),          # an array field
    ('mzparms', nfloat32[:]),          # an array field
    ('fxparms', nfloat32[:]),          # an array field
    ('K', nfloat32),               # a simple scalar field
    ('alpha', nfloat32), 
    ('alpha_deg', nfloat32),
    ('kappa', nfloat32), 
    ('camb', nfloat32),               # a simple scalar field
    ('Fz', nfloat32), 
    ('Fy', nfloat32), 
    ('Fx', nfloat32), 
    ('fric_x_max', nfloat32), 
    ('fric_y_max', nfloat32), 
    ('C',nfloat32),
    ('D',nfloat32),
    ('C_alpha',nfloat32),
    ('E',nfloat32),
    ('Sh',nfloat32),
    ('Sv',nfloat32),
    ('delB',nfloat32),
    ('B',nfloat32),
    ('BCDx',nfloat32),
    ('Bx',nfloat32),
    ('Dx',nfloat32),
    ('Cx',nfloat32),
    ('Ex',nfloat32),
    ('C_alpha',nfloat32),
    ('phix',nfloat32),
    ('phi',nfloat32),
    ('Fx_comb',nfloat32),
    ('Fy_comb',nfloat32)
]

import numpy as np

def pow(number,power):
    return number**power


@jitclass(spec)
class PacejkaTire(object):
    def __init__(self,iFz,ifric_x_max,ifric_y_max):
        #parameters explained: http://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
        self.fyparms = np.array([-22.1, 1011, 1078, 1.82, .208, -0.00, -0.354, .707, .028, 0.0, 14.8, .022, 0.0],dtype=np.float32);
        self.mzparms = np.array([-2.72, -2.28, -1.86, -2.73, 0.110, -0.07, .643, -4.04, 0.015, -0.066, .945, 0.03, 0.07],dtype=np.float32);
        self.fxparms = np.array([-21.3, 1144, 49.6, 226, 0.069, -0.006, 0.056, .486],dtype=np.float32);
        #these parameters were taken from: 
        self.K=50.0
        self.alpha = 0
        self.kappa = 0
        self.camb = 0
        self.Fz = iFz
        #self.updateFyparams(self.Fz);
        self.fric_x_max = ifric_x_max
        self.fric_y_max = ifric_y_max
        self.C=0
        self.D=0
        self.C_alpha=0
        self.E=0
        self.Sh=0
        self.Sv=0
        self.delB=0
        self.B=0
        self.C_alpha=0
        self.phix=0
        self.Fy,self.Fx,self.Fx_comb=0,0,0
        self.phi=0
        self.Fy_comb=0
        self.Cx,self.Bx,self.Ex,self.BCDx=0,0,0,0
        
    #@property
    def findKappa(self, iFz, ialpha, iFx):
        #kappa = np.linspace(np.float32(-2.0),np.float32(2.0),100,dtype=np.float32)
        kappa = np.arange(-2.,2.,0.01)
        Fx = np.zeros(len(kappa),dtype=np.float32)
        for ind in range(0,len(kappa)):
          Fx[ind] = self.calcFx(5000,kappa[ind],ialpha)
        maxfx = np.max(Fx)
        minfx = np.min(Fx)
        #print Fx
        maxfx_kappa = np.where(Fx==maxfx)[0][0]
        minfx_kappa = np.where(Fx==minfx)[0][0]

        newkappa = kappa[minfx_kappa:maxfx_kappa]
        newfx = Fx[minfx_kappa:maxfx_kappa]
        fxincreasing = np.all(np.diff(newfx) > 0)
        # mykappa = np.interp(iFx,Fx,kappa,right=maxfx_kappa,left=-maxfx_kappa)
        if fxincreasing:
          mykappa = np.interp(iFx,newfx,newkappa)
        else: #probably means we're sliding
          mykappa = maxfx_kappa
        return mykappa

    def updateFyparams(self,iFz):
        self.Fz = iFz;
        #use braking friction to cap y friction
        #allowable_acceleration = sqrt((self.fxb*self.g)**2*(1-lat_accel_here**2/((self.g*self.fy)**2))) #self is the magnitude of the acceleration the tires can tolerate here
        self.C=1.3;#sqrt(pow(fric_y_max,2)*(1-pow(fric_x,2)/pow(fric_x_max,2)));#self is the ultimate friction. uses friction ellipse to scale.
        self.D=self.fyparms[0]*(self.Fz*.001)**2+self.fyparms[1]*self.Fz*.001;
        self.C_alpha = self.fyparms[2]*np.sin(self.fyparms[3]*np.arctan(self.fyparms[4]*self.Fz));
        self.B = self.C_alpha/(self.C*self.D);
        self.E = self.fyparms[5]*(self.Fz*.001)**2+self.fyparms[6]*self.Fz*.001+self.fyparms[7];
        self.Sh = -self.fyparms[8]*self.camb*np.pi/180;
        self.Sv = self.camb*(-self.fyparms[9]*np.pi/180*(self.Fz*.001)**2+-self.fyparms[10]*np.pi/180*self.Fz*.001);
        self.delB = -self.fyparms[11]*abs(self.camb)*self.B*np.pi/180;
        self.B = self.B+self.delB;


    def calcFy(self,iFz, ialpha, ikappa, icamb):
        self.kappa = ikappa;
        self.Fz = iFz;
        self.alpha = ialpha;
        self.alpha_deg = self.alpha*180/np.pi;
        self.camb = icamb;
        self.updateFyparams(self.Fz);
        self.phi = (1-self.E)*(self.alpha_deg+self.Sh)+self.E/(self.B)*arctan(self.B*(self.alpha_deg+self.Sh));
        self.Fy = self.D*sin(self.C*arctan(self.B*self.phi))+self.Sv;
        #now we adjust for longitudinal slip (per adams pacejka2002 manual)
        #we will use friction ellipse instead of "cosine weighting functions"
        if (abs(self.kappa)>0 or abs(self.alpha)>0 or abs(self.camb)>0):
              #self is not part of Pacejka's original formulation, but was developed by MSC
              kappac= self.kappa;
              alphac = self.alpha+self.Sh+self.Sv/self.K;#NOTE: THESE K /100 are just made up!! fix?
              alphastar = sin(alphac);
              beta = np.arccos(abs(kappac)/sqrt((kappac)**2+(alphastar)**2));#NOTE had abs(kappac)
              muy_now = (self.Fy-self.Sv)/self.Fz;
              mux_max = self.Dx/self.Fz;
              muy = np.abs(np.tan(beta)/np.sqrt((1/mux_max)**2+(np.tan(beta)/(muy_now+.01))**2));
              self.Fy_comb = self.Fy*muy/np.abs(0.001+muy_now);
            
        else:
          self.Fy_comb = self.Fy;

        return -self.Fy_comb;#combined case cornering force

    def calcFy_xforce(self,iFz, ialpha, iFx, icamb):
        self.kappa = self.findKappa(iFz,ialpha,iFx);
        self.Fz = iFz;
        self.alpha = ialpha;
        self.alpha_deg = self.alpha*180/np.pi;
        self.camb = icamb;
        self.updateFyparams(self.Fz);
        self.phi = (1-self.E)*(self.alpha_deg+self.Sh)+self.E/(self.B)*np.arctan(self.B*(self.alpha_deg+self.Sh));
        self.Fy = self.D*np.sin(self.C*np.arctan(self.B*self.phi))+self.Sv;
        #now we adjust for longitudinal slip (per adams pacejka2002 manual)
        #we will use friction ellipse instead of "cosine weighting functions"
        if (abs(self.kappa)>0 or abs(self.alpha)>0 or abs(self.camb)>0):
              #self is not part of Pacejka's original formulation, but was developed by MSC
              kappac= self.kappa;
              alphac = self.alpha+self.Sh+self.Sv/self.K;#NOTE: THESE K /100 are just made up!! fix?
              alphastar = np.sin(alphac);
              beta = np.arccos(abs(kappac)/np.sqrt((kappac)**2+(alphastar)**2));#NOTE had abs(kappac)
              muy_now = (self.Fy-self.Sv)/self.Fz;
              mux_max = self.Dx/self.Fz;
              muy = np.abs(np.tan(beta)/np.sqrt((1/mux_max)**2+(np.tan(beta)/(muy_now+.01))**2));
              self.Fy_comb = self.Fy*muy/np.abs(0.001+muy_now);
            
        else:
          self.Fy_comb = self.Fy;

        return -self.Fy_comb;#combined case cornering force

    def updateFxparams(self,iFz):
        self.Fz = iFz;
        self.Cx = 1.65;#fric_x_max;#we scale the lat accel, so leave self alone??
        self.Dx = self.fxparms[0]*(self.Fz*.001)**2+self.fxparms[1]*self.Fz*.001;
        self.BCDx = (self.fxparms[2]*(self.Fz*.001)**2+self.fxparms[3]*self.Fz*.001)/(np.exp(self.fxparms[4]*.001*self.Fz));#
        #println("BCDx= "+exp(-fxparms[4]*Fz));
        self.Bx = self.BCDx/(self.Cx*self.Dx);
        #E = fyparms[5]*pow(Fz*.001, 2)+fyparms[6]*Fz*.001+fyparms[7];
        self.Ex = self.fxparms[5]*(self.Fz*.001)**2+self.fxparms[6]*self.Fz*.001+self.fxparms[7];

    def calcFx(self,iFz, ikappa, ialpha):
        self.alpha = ialpha;
        self.Fz = iFz;
        self.kappa = ikappa;
        self.updateFxparams(self.Fz);
        self.updateFyparams(self.Fz);
        self.phix = (1-self.Ex)*self.kappa*100+self.Ex/(self.Bx+.001)*np.arctan(self.Bx*self.kappa*100);
        #println("Bx, Phix= "+str(Bx)+","+str((1-Ex)*kappa*100));
        self.Fx = self.Dx*np.sin(self.Cx*np.arctan(self.Bx*self.phix));
        #print self.Fx
        if (np.abs(self.alpha)>0 or np.abs(self.kappa)>0 or np.abs(self.camb)>0):
          #self is not part of Pacejka's original formulation, but was developed by MSC
          kappac= self.kappa;
          alphac = self.alpha+self.Sh+self.Sv/self.K;
          alphastar = np.sin(alphac);
          thecosine = np.abs(kappac)/np.sqrt(kappac**2+alphastar**2)
          #print thecosine
          if(np.abs(thecosine)>1):
            #print "THE COSINE WAS: "+str(thecosine)
            thecosine = np.sign(thecosine)*1.0
          elif (np.isnan(thecosine)):
            #print "found a nan cosine"
            thecosine = 1.0

          beta = np.arccos(thecosine);#NOTE had abs(kappac) in num
         # print beta
          mux_now = (self.Fx)/self.Fz;#
          muy_max = self.D/self.Fz;
          mux = 1/np.sqrt(.001+1/(mux_now+.001)**2+(np.tan(beta)/muy_max)** 2);

          self.Fx_comb = self.Fx*mux/np.abs(mux_now+.001);
        else :
          self.Fx_comb=self.Fx;
        return self.Fx_comb;#combined case cornering force


def main():
  t = PacejkaTire(5000.,1.1,1.1)
  kappa = np.linspace(-2,2,1000)
  Fx = np.zeros(1000)
  for ind in range(0,len(kappa)):
    Fx[ind] = t.calcFx(5000,kappa[ind],0)

  figure()
  plot(kappa,Fx,'k')
  xlabel('longitudinal slip fraction')
  ylabel('Longitudinal Force (N)')

  fxvec = [0,2500,3000,4000,4999]
  Fy = np.zeros((5,1000))
  alphas = np.linspace(-18*np.pi/180,18*np.pi/180,1000)
  for ind in range(0,len(fxvec)):
    #calcFy_xforce(self,iFz, ialpha, iFx, icamb):
    for ind2 in range(0,len(alphas)):
      Fy[ind,ind2]=t.calcFy_xforce(5000,alphas[ind2],fxvec[ind],0)
      #print(ind,ind2)

  #print(alphas.shape,Fy.shape)
  figure()
  for ind in range(0,len(fxvec)):
    plot(alphas,Fy[ind,:])
  # plot(dot(array([[1],[1],[1],[1],[1]]),array([alphas])),Fy)

  #show()

if __name__ == '__main__':
  print("comnp.piling and running for first time...")
  start = time.time()
  main()
  end = time.time()
  print("Elapsed (with comnp.pilation) = %s" % (end - start))

  start = time.time()
  main()
  end = time.time()
  print("Elapsed (after comnp.pilation) = %s" % (end - start))
