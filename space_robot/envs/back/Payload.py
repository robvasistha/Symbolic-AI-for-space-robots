import numpy as np
import random

class Payload():
    def __init__(self):
        self.loc = []
        self.mass = []
        self.size = []

    def set_location(self, pose):
        self.loc = pose

    def get_location(self):
        return self.loc

    def set_mass(self, mass):
        self.mass = mass

    def set_size(self, size):
        self.size = size

    def get_size(self):
        return self.size

    def generate_random(self):
        loc = np.zeros([6])
        loc[0] = random.randrange(10, 90)*(-0.01)
        loc[1] = random.randrange(10, 90)*(0.01)
        loc[2] = random.randrange(100, 180)*(0.01)
        for i in range (3):
            #loc[3+i] = random.randrange(-10, 10)
            loc[3+i] = 0
        self.loc = loc
        self.mass = random.randrange(1,5)

        size = np.zeros([3])
        x = random.randrange(10, 30)
        for i in range(3):
            size[i] = x
        self.size = np.array([0.2, 0.2, 0.2])
        return self.size, self.loc, self.mass