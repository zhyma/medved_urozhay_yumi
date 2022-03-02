#!/usr/bin/env python3

from math import pi, sqrt
import numpy as np

class path_generator():

    def __init__(self):
        ...

    def generate_spiral(self, sprial_params, gripper_states):
        path = []
        # # test to plot a line.
        # for i in range(30):
        #     path.append([0.3, i/100, 0.4])

        
        # s is the center of the rod
        # x (front-back), y (left-right), z (up-down).
        # x and z are used for spiral, y is used for advance
        # s = spiral_params
        s = [0.3, 0.1, 0.4]
        # g is the gripper's starting position
        # g = gripper_states
        g = [0.6, 0.1, 0.42]
        # starting angle theta_0
        t_0 = 0.2
        # ending angle theta_end
        t_e = 0.2+pi*2
        n_samples = 20
        n = -1/2
        # offset theta_offset
        t_os = 0

        # r = a\theta^(-1/2)
        r_0 = np.sqrt((s[0]-g[0])**2+(s[2]-g[2])**2)
        a = r_0/np.power(t_0, n)
        for i in range(n_samples):
            dt = (t_e-t_0)*i/n_samples
            r = a*np.power((t_0+dt), n)
            t = -dt + t_os
            x = r*np.cos(t) + s[0]
            y = g[1] + 0.005*i
            z = r*np.sin(t) + s[2]
            path.append([x,y,z])
        
        return path

    def generate_line(self, start = [0, 0, 0], stop=[0, 0, 0]):
        path = []

        dist = sqrt((stop[0]-start[0])**2 + (stop[1]-start[1])**2 + (stop[2]-start[2])**2)
        no_of_points = int(dist/0.05) + 1
        print('number of waypoints are: {0}'.format(no_of_points))
        dx = (stop[0]-start[0])/(no_of_points - 1)
        dy = (stop[1]-start[1])/(no_of_points - 1)
        dz = (stop[2]-start[2])/(no_of_points - 1)

        for i in range(no_of_points):
            path.append([start[0]+i*dx, start[1]+i*dx, start[2]+i*dx])

        return path