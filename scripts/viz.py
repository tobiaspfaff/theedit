#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
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
+
print 'Load complete'

# display
fig = plt.figure()
ax = plt.axes(xlim=(-1.2,1.2), ylim=(-1.2,1.2))
particles = ax.scatter([],[],edgecolors=None,s=100)
ax.add_patch(patches.Rectangle((-1,-1),2,2,fill=False))

def init_plot():
    #particles.set_data([],[])
    return particles,

def le_plot(idx):    
    print 'Frame %d' % idx
    particles.set_offsets(data[idx])
    particles.set_array(np.linspace(0,0.1,len(data[idx])))
    return particles,


anim = animation.FuncAnimation(fig, le_plot, init_func=init_plot, frames=len(data), blit=False, interval=20, repeat=False)
plt.show()