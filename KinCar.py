from numpy import array,sin,cos

class KinCar:
    def __init__(self,dt=0.01,a = 0.8,b = 1.5,Y=0,V=0,X=0,U=0,Psi=0,r=0):
        self.dt,self.a,self.b,self.Y,self.V,self.X,self.U,self.Psi,self.r=dt,a,b,Y,V,X,U,Psi,r
        self.rtau = 0.05 #just a guess...
    def state_eq(self,steer,Udot):
        Ydot = self.U*sin(self.Psi)
        # Ydot = self.U*(self.Psi)
        Vdot = 0
        Xdot = self.U*cos(self.Psi)
        Udot = Udot #this is redundant. Just showing that longitudinal acc is an input
        Psidot = self.r
        goal_r = self.U*steer/(self.a+self.b)
        rdot = 1/self.rtau*(goal_r-self.r)
        return Ydot,Vdot,Xdot,Udot,Psidot,goal_r,rdot

    def euler_update(self,steer=0,brake=0,gas=0):
        anet = (gas-brake)*9.81#get this as an acceleration
        Ydot,Vdot,Xdot,Udot,Psidot,goal_r,rdot = self.state_eq(steer,anet)
        self.Y+=Ydot*self.dt
        self.V+=Vdot*self.dt
        self.X+=Xdot*self.dt
        self.U+=Udot*self.dt
        self.Psi+=Psidot*self.dt
        self.r+=rdot*self.dt

        return array([self.Y,self.V,self.X,self.U,self.Psi,self.r]),array([Ydot,Vdot,Xdot,Udot,Psidot,rdot]),steer