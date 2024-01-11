import csv
import numpy as np
import matplotlib.pyplot as plt
import mio as M

# this_file = csv.reader(open('Simulation_Data.csv','r'),delimiter=',')
# data = []
# for row in this_file:
#     data.append(row)
#     print(row)
# data_np = np.asarray(data)
data = M.dlmread('Simulation_Data.csv',',')
time = data[:,0]
ptp_uncontrolled = data[:,1:4]
ptp_controlled = data[:,4:7]

plt.figure()
plt.plot(time,ptp_uncontrolled,'b-')

plt.figure()
plt.plot(time,ptp_controlled,'b-')
plt.show()

