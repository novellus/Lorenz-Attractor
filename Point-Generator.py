class LorenzGenerator:
	x=1
	y=1
	z=1
	sigma=10
	rho=28
	beta=8.0/3
	dt=.01

	def __init__(self,sigma=10,rho=28,beta=8.0/3):
		self.sigma=sigma
		self.rho=rho
		self.beta=beta

	def start(self,x0=1,y0=1,z0=1,timeStep=0.01):
		self.x=x0
		self.y=y0
		self.z=z0
		self.dt=timeStep

	def setTimestep(self,timeStep):
		self.dt=timeStep

	def getNextPoint(self):
		dx=(self.sigma*(self.y-self.x))*self.dt
		dy=(self.x*(self.rho-self.z)-self.y)*self.dt
		dz=(self.x*self.y-self.beta*self.z)*self.dt
		self.x+=dx
		self.y+=dy
		self.z+=dz
		return (self.x,self.y,self.z)

		dx=(sigma*(y-x))*dt
		dy=(x*(rho-z)-y)*dt
		dz=(x*y-beta*z)*dt
		x+=dx
		y+=dy
		z+=dz
		return (x,y,z)
