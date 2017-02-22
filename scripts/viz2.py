#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import sys, os, time

x=[]
y=[]
fe_y=[]
int_y=[]
t0=[]
t1=[]
with open('out/x.txt', 'r') as fp:
    for line in fp:
        tok = line.split(' ')
        t0.append(float(tok[0]))
        x.append(float(tok[1]))
        y.append(float(tok[2]))

with open('out/res.txt', 'r') as fp:
    for line in fp:
        tok = line.split(' ')
        t1.append(float(tok[0]))
        fe_y.append(float(tok[2]))
        int_y.append(float(tok[4]))
        
plt.subplot(2,1,1)
plt.plot(t0,y)
plt.subplot(2,1,2)
h_fe, = plt.plot(t1,fe_y,label='FE (y)')
h_int, = plt.plot(t1,int_y,label='Int (y)')
plt.legend(handles=[h_fe,h_int])
plt.show()