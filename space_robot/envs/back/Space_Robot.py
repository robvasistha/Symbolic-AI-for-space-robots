import numpy as np
import random

##This stores all parameters to do with the sizing of the robot.
###Any chances to parameters go through this module.

class Space_Robot():
    def __init__ (self, n):
        self.r =  np.array([[0.03], [0.02]])
        self.ee = np.array([[0.0378, 0.0376, 0.0376],[0.0376, 0.0379, 0.0376],[0.0376,0.0376,0.038]])
        self.density = 2.710
        self.l = []
        self.ff = []
        self.m = []
        self.s = []
        self.b = []
        self.dof = n

    def set_parameters(self):
        ff = np.array([0.5, 0.5, 0.5, 100])
        if self.dof == 4:
            l = np.array([0.2, 0.2, 0.2, 0.05])
        if self.dof == 5:
            l = np.array([0.2, 0.2, 0.2, 0.2, 0.05])
        if self.dof == 6:
            l = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.05])
        self.lengths = l
        if self.dof == 4:
            self.l = np.array([[0, 0, 0], [l[0], 0, 0], [l[1], 0, 0], [l[2], 0, 0], [l[3], 0, 0]])
        if self.dof == 5:
            self.l = np.array([[0, 0, 0], [l[0], 0, 0], [l[1], 0, 0], [l[2], 0, 0], [l[3], 0, 0], [l[4], 0, 0]])
        if self.dof == 6:
            self.l = np.array([[0, 0, 0], [l[0], 0, 0], [l[1], 0, 0], [l[2], 0, 0], [l[3], 0, 0], [l[4], 0, 0], [l[5], 0, 0]])
        self.ff = ff
        self.s = (self.l) / 2
        self.s[0][2] = ff[2] / 2
        self.b = (self.l) / 2
        M = np.zeros([self.dof+1,self.dof+1])
        for i in range (self.dof+1):
            for k in range (3):
                M[i][k] = (np.pi*self.l[i][k]*(np.square(self.r[0])-np.square(self.r[1]))*4)*self.density
        M[0][0] = ff[3]
        self.m = M.max(axis=1)

    def change_formfactor(self, ff):
        self.m[0] = ff[3]
        self.s[0][2] = ff[2] / 2

    def update_dof(self, dof):
        self.dof = dof

    def set_linklengths(self, l):
        self.lengths = l
        if self.dof == 4:
            self.l = np.array([[0, 0, 0], [l[0], 0, 0], [l[1], 0, 0], [l[2], 0, 0], [l[3], 0, 0]])
        if self.dof == 5:
            self.l = np.array([[0, 0, 0], [l[0], 0, 0], [l[1], 0, 0], [l[2], 0, 0], [l[3], 0, 0], [l[4], 0, 0]])
        if self.dof == 6:
            self.l = np.array([[0, 0, 0], [l[0], 0, 0], [l[1], 0, 0], [l[2], 0, 0], [l[3], 0, 0], [l[4], 0, 0], [l[5], 0, 0]])
        self.s = (self.l) / 2
        self.s[0][2] = self.ff[2] / 2
        self.b = (self.l) / 2
        M = np.zeros([self.dof+1,self.dof+1])
        for i in range (self.dof+1):
            for k in range (3):
                M[i][k] = (np.pi*self.l[i][k]*(np.square(self.r[0])-np.square(self.r[1]))*4)*self.density
        M[0][0] = self.ff[3]
        self.m = M.max(axis=1)

    def get_link_lengths_matrix(self):
        return self.l

    def get_link_lengths(self):
        return self.lengths

    def get_link_mass(self):
        return self.m

    def get_dof(self):
        return self.dof

    def get_s(self):
        return self.s

    def get_b(self):
        return self.b

    def get_r(self):
        return self.r

    def get_base_dimensions(self):
        return self.ff[0:3]

    def get_ee_inertia(self):
        return self.ee

    def get_base_mass(self):
        return self.m[0]
