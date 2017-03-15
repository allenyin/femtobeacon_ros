import numpy as np
import matplotlib.pyplot as plt

f = open('fem_cha_2.log', 'r')
readings = [[],[]]  # first is beacon, second is chair

n = 0
lookfor = 2

def mapTo360(ang):
    return (ang+360)%360

with open('fem_cha_2.log') as f:
    for line in f:
        f_parts = [float(i) for i in line.split()]
        if len(f_parts) != 2:
            continue

        if f_parts[0] == lookfor:
            readings[lookfor-1].append(f_parts[1])
            lookfor = lookfor%2 + 1

beacon = np.array(readings[0])
chair = np.array(readings[1])

# plot all the angles
plt.figure()
plt.plot(mapTo360(chair), 'b')
plt.plot(mapTo360(-beacon), 'r')
plt.legend(['chair', 'beacon'])
plt.show(block=False)
