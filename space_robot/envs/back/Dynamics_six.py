from space_robot.envs.Matricies.six_dof.C_m import *
from space_robot.envs.Matricies.six_dof.C_vm import *
from space_robot.envs.Matricies.six_dof.C_wm import *
from space_robot.envs.Matricies.six_dof.C_sc_v import *
from space_robot.envs.Matricies.six_dof.C_sc_w import *
from space_robot.envs.Matricies.six_dof.D_m import *
from space_robot.envs.Matricies.six_dof.D_v import *
from space_robot.envs.Matricies.six_dof.D_vm import *
from space_robot.envs.Matricies.six_dof.D_vw import *
from space_robot.envs.Matricies.six_dof.D_w import *
from space_robot.envs.Matricies.six_dof.D_wm import *
from space_robot.envs.Matricies.six_dof.D_wv import *
import numpy as np
from numpy.linalg import inv
from scipy.integrate import solve_ivp

class Dynamics_six():
    def __init__(self):
        self.robot = []

    def reset(self, space_robot):
        self.n = space_robot.get_dof()
        self.m = space_robot.get_link_mass()
        self.l = space_robot.get_link_lengths_matrix()
        self.s = space_robot.get_s()
        self.b = space_robot.get_b()
        self.R = space_robot.get_r()
        self.dimen = space_robot.get_base_dimensions()
        self.ee = space_robot.get_ee_inertia()

        state_reset = np.zeros([2*(self.n+6)])
        return(state_reset)

    def update(self, space_robot):
        self.n = space_robot.get_n()
        self.m = space_robot.get_link_mass()
        self.l = space_robot.get_link_lengths_matrix()
        self.s = space_robot.get_s()
        self.b = space_robot.get_b()
        self.R = space_robot.get_r()
        self.dimen = space_robot.get_base_dimensions()
        self.ee = space_robot.get_ee_inertia()

    def C_matrix(self, phi, phi_d, theta, theta_d):
        Cm = C_m(self.m, phi, theta, theta_d, self.l, self.s, self.b, self.R, self.dimen, self.ee)
        Cvm = C_vm(self.m, phi, theta, theta_d, self.l, self.s, self.b, self.R, self.dimen, self.ee)
        Cwm = C_wm(self.m, phi, theta, theta_d, self.l, self.s, self.b, self.R, self.dimen, self.ee)
        Cscv = C_sc_v(self.m, phi, phi_d, theta, theta_d, self.l, self.s, self.b, self.R, self.dimen, self.ee)
        Cscw = C_sc_w(self.m, phi, phi_d, theta, theta_d, self.l, self.s, self.b, self.R, self.dimen, self.ee)

        a = np.zeros((3,3))
        c1 = np.concatenate((a, a))
        c2 = np.concatenate((Cscv,Cscw))
        c3 = np.concatenate((Cvm,Cwm))
        c4 = np.concatenate((c1, c2, c3), axis = 1)
        c5 = np.concatenate((Cvm.T, Cwm.T, Cm), axis = 1)
        C = np.concatenate((c4, c5))
        return C

    def D_matrix(self, phi, theta):
        Dm = D_m(self.m, phi, theta, self.l, self.s, self.b, self.R, self.ee)
        Dvm = D_vm(self.m, phi, theta, self.l, self.s, self.b, self.R, self.dimen, self.ee)
        Dwm = D_wm(self.m, phi, theta, self.l, self.s, self.b, self.R, self.dimen, self.ee)
        Dv = D_v(self.m)
        Dw = D_w(self.m, phi, theta, self.l, self.s, self.b, self.R, self.dimen, self.ee)
        Dvw = D_vw(self.m, phi, theta, self.l, self.s, self.b, self.R, self.dimen, self.ee)
        Dwv = D_wv(self.m, phi, theta, self.l, self.s, self.b, self.R, self.dimen, self.ee)

        d1 = np.concatenate((Dv,Dwv))
        d2 = np.concatenate((Dvw,Dw))
        d3 = np.concatenate((Dvm,Dwm))
        d4 = np.concatenate((d1,d2,d3), axis=1)
        d5 = np.concatenate((Dvm.T,Dwm.T,Dm), axis=1)
        D = np.concatenate((d4,d5))
        return D

    def space_robot_step(self, action, part_state):
        phi = part_state[3:6]
        theta = part_state[6:12]
        phi_d = part_state[15:18]
        theta_d = part_state[18:24]
        C = self.C_matrix(phi, phi_d, theta, theta_d)
        D = self.D_matrix(phi, theta)

        def dxd_dt(t, x, action, D, C):
            act = action
            x1 = x[0:12]
            x2 = x[12:24]
            dx1d_dt = x2
            dx2d_dt = inv(D).dot(act - C.dot(x2))
            dxd_dt = np.append(dx1d_dt, dx2d_dt)
            #print(act)
            return dxd_dt

        U0 = part_state
        new_state = solve_ivp(lambda t, x: dxd_dt(t, x, action, D, C), [0,1], U0)
        N_s = new_state['y'][:, -1]
        return N_s