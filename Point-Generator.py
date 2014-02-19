class LorenzGenerator:
	x=0
	y=0
	z=0
	sigma=10
	rho=28
	beta=8.0/3
	timeStep=.01

	def __init__(self,sigma=10,rho=28,beta=8.0/3):
		self.sigma=sigma
		self.rho=rho
		self.beta=beta

	def start(self,x0=0,y0=0,z0=0,timeStep=0.01):
		self.x=x0
		self.y=y0
		self.z=z0
		self.timeStep=timeStep

	def setTimestep(self,timeStep):
		self.timeStep=timeStep

	def getNextPoint(self):
		dx=(sigma*(y-x))*dt
		dy=(x*(rho-z)-y)*dt
		dz=(x*y-beta*z)*dt
		x+=dx
		y+=dy
		z+=dz
		return (x,y,z)
