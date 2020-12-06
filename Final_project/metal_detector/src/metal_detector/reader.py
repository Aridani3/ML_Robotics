import rospy
import numpy as np    
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score
import scipy 
import rospkg



class SensorModel:
    def __init__(self):
        self.sensorModelmean = None
        self.sensorModelstd = None
        self.file_path = rospkg.RosPack().get_path('metal_detector')+'/src/metal_detector/metal_detector_data.txt' # a modifier par la suite
        self.min_m = 0.15
        self.data = None

    def read_file(self, filepath):
        d = []
        m = []
        with open(filepath) as f:
            line = f.readline()
            while line:
                data = line.strip().split(' ')
                d.append(float(data[0]))
                m.append(float(data[1]))
                line = f.readline()
        return np.array(d), np.array(m)
    
    def get_model(self):

        d, m = self.read_file(self.file_path)
        n = np.round(m/0.05)

        #get data for which measurement > min_m
        self.data = np.vstack((m,d))
        self.data = self.data[:,self.data[0] >= self.min_m ]

        #Create a histogram of data with step 0.05 over measurements
        hist_data = np.vstack((n,d))
        hist_data = hist_data[:,hist_data[0,:].argsort()]
        hist_data = hist_data[:,hist_data[0] > np.round(self.min_m/0.05) ]

        #get mean and std of each bar of histogram
        x = np.unique(hist_data[0])*0.05
        hist_data = np.split(hist_data[1,:], np.unique(hist_data[0, :], return_index=True)[1][1:])
        mean = np.array([np.mean(k) for k in hist_data])
        std = np.array([np.std(k) for k in hist_data])

        #polynomial regression over histogram's stds
        self.sensorModelstd = np.poly1d(np.polyfit(x, std, 4))

        #polynomial regression over all data points
        p, res, _, _, _ = np.polyfit(self.data[0], self.data[1], 4, full=True)
        self.sensorModelmean = np.poly1d(p)
        err = np.sqrt(res/(len(d)-1)) # std for polyfit + won't use it 

"""
Sensor = SensorModel()
Sensor.get_model()
model = Sensor.sensorModelmean
p_std = Sensor.sensorModelstd

x = np.linspace(Sensor.min_m-0.05, 1, 100)

###PLOTS
#poly reg
plt.plot(x, model(x), 'r')
#std_reg
plt.plot(x, model(x)+2*p_std(x), 'k')
plt.plot(x, model(x)-2*p_std(x), 'k')
#data
plt.plot(Sensor.data[0], Sensor.data[1], 'b.', markersize = 2)

plt.xlabel('measurement')
plt.ylabel('distance [m]')

plt.show()
"""
