import numpy as np
from numpy import genfromtxt

gps_data = np.array(genfromtxt('Graph1.txt', delimiter=',', skip_header=1, dtype=np.float64))
acc_data = np.array(genfromtxt('Graph2.txt', delimiter=',', skip_header=1, dtype=np.float64))

gps_sd = np.std(gps_data, axis=0, dtype=np.float64)[1]
acc_sd = np.std(acc_data, axis=0, dtype=np.float64)[1]

# quality of output is not so good
print("GPS Standard Deviation: {}".format(gps_sd))
print("ACC Standard Deviation: {}".format(acc_sd))
