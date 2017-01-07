#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, os, time

def load(filename):
    with open(filename, 'r') as fp:
        num = int(fp.readline())
        pos = np.ndarray(shape=(num,2))
        for i in range(num):
            line = fp.readline().split()
            if line:
                pos[i,:] = [float(line[0]), float(line[1])]
    return pos

# main
if len(sys.argv) != 2:
    print 'viz.py [out dir]'
    sys.exit(1)


# load data
data = []
outdir = sys.argv[1]
for i in range(100000):
    filename = outdir + '/%05d.txt' % i
    if not os.path.exists(filename):
        break

    pos = load(filename)
    data.append(pos)

print 'Load complete'

# display
fig = plt.figure()
ax = plt.axes(xlim=(-10,10), ylim=(-10,10))
particles, = ax.plot([],[],'bo',ms=6)

def init_plot():
    particles.set_data([],[])
    return particles,

def le_plot(idx):    
    print 'Frame %d' % idx
    particles.set_data(data[idx][:,0],data[idx][:,1])
    return particles,


anim = animation.FuncAnimation(fig, le_plot, init_func=init_plot, frames=len(data), blit=False, interval=20, repeat=False)
plt.show()