import rospy
import numpy as np



class ParticleFilter():
    """
    Implmentation of the particle filter / Bayes filter for metal Detector
    """
    def __init__(self, sensorModelmean, sensorModelstd, initial_pose, initial_measurement, Nparticules=10):
        """
        """
        self.Np = Nparticules
        self.particles = np.array([])
        self.Weights = np.ones(self.Np) / self.Np # Weights of the particles
        self.samples = np.arange(self.Np) # Allows for the identification of the particles
        self.initial_pose = np.array(initial_pose) # [x, y]
        self.initial_measurement = initial_measurement
        self.sensorModelmean = sensorModelmean
        self.sensorModelstd = sensorModelstd

        self.initialize()  

    def initialize(self):
        """
        Initializes the particle filter
        """      
        d = np.random.normal(self.sensorModelmean(self.initial_measurement), 
                             self.sensorModelstd(self.initial_measurement), self.Np) 
        a = np.random.uniform(0,2*np.pi, self.Np)

        self.particles = self.initial_pose.reshape(-1,1) + np.vstack((d*np.cos(a), d*np.sin(a)))
                    

    def Gaussian(self, x, mu, sig):
        return np.exp(-np.power(x - mu, 2) / (2 * np.power(sig, 2)))

    def computeWeights(self, Z, pose, beta=2):
        """
        """
        d = np.hypot(self.particles[0]-pose[0], self.particles[1] - pose[1])
        W = self.Gaussian(d, self.sensorModelmean(Z), self.sensorModelstd(Z))
        W = np.exp(2*W) - 1
        return W / np.sum(W)

    def update(self, Z, pose, dx, dy):
        """
        """   
        W = self.computeWeights(Z, pose)

        self.samples = np.random.choice(self.Np, self.Np, p=list(W))
        self.particles = np.array([self.particles[:,k].copy() for k in self.samples]).T
        self.Weights = np.take(W, self.samples, axis=0).reshape(-1)

    def getBestParticle(self):
        """
        Returns the position that corresponds to the particle with the 
        highest weigth
        """
        idx = np.argmax(self.Weights)

        return self.particles[:,idx]

