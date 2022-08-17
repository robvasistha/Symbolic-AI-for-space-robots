import numpy as np

class change_frame:
    def __init__(self):
        self.world = world = np.array([0, 0, 0, 0, 0, 0])

    def c(self, alpha):
        return np.cos(alpha)

    def s(self, alpha):
        return np.sin(alpha)


    def make_transform(self, state):
        # This function makes a rotation matrix and translation matrix between the input vector and the origin of the world.
        # Base on a 3-2-1 rotation
        x, y, z = state[0], state[1], state[2]
        alpha, beta, gamma = state[3], state[4], state[5]

        rot = np.array([[self.c(beta)*self.c(gamma), self.c(beta)*self.s(gamma), -self.s(beta), 0],
                        [-self.c(alpha)*self.s(gamma)+self.s(alpha)*self.s(beta)*self.c(gamma), self.c(alpha)*self.c(gamma)+self.s(alpha)*self.s(beta)*self.s(gamma), self.s(alpha)*self.c(beta),0],
                        [self.s(alpha)*self.s(gamma)+self.c(alpha)*self.s(beta)*self.c(gamma), -self.s(alpha)*self.c(gamma)+self.c(alpha)*self.s(beta)*self.s(gamma), self.c(alpha)*self.c(beta), 0],
                        [0, 0, 0, 1]])

        trans = np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
        return rot, trans

    def decompose(self, T):
        # Decomposes a transformation matrix into euler angles and vector position.
        alpha = np.arctan2(T[1,2], T[2,2])
        cos2 = np.sqrt(np.square(T[0,0]) +  np.square(T[0,1]))
        beta = np.arctan2(-T[0,2], cos2)

        s1 = self.s(alpha)
        c1 = self.c(alpha)

        A = s1*T[2,0] - c1*T[1,0]
        B = c1*T[1,1] - s1*T[2,1]

        gamma = np.arctan2(A,B)

        x = T[0,3]
        y = T[1,3]
        z = T[2,3]

        return np.array([x, y, z, alpha, beta, gamma])

    def convert(self, state):
        # Takes the full state and converts the payload position to be in terms of the reference frame of the base and the velocity of the base to be in terms of rate of change of body axis as oppose to
        # the rate of change of euler angles.
        robot = state[0:6]
        payload = state[20:26]

        # gives rotation and translation matricies between the robot/payload and the world origin
        rr, tr = self.make_transform(robot)
        rp, tp = self.make_transform(payload)

        # Assume the rotation happens prior to translation
        T_wrT = np.linalg.inv(tr).dot(np.transpose(rr))
        T_wp = rp.dot(tp)
        final = T_wrT.dot(T_wp)
        payload_pos_from_base = self.decompose(final)

        # Now need to convert the velocity of the base.
        new_vel_lin, new_vel_ang = self.veloctiy_change(state)
        new_state = state
        new_state[0:6] = np.array([0,0,0,0,0,0])
        new_state[10:13] = new_vel_lin
        new_state[13:16] = new_vel_ang
        new_state[20:26] = payload_pos_from_base
        return new_state

    def veloctiy_change(self, state):
        # Converts velocity of the base to be in terms of rate of change of body axis as oppose tothe rate of change of euler angles.
        x, y, z = state[0:3]
        alpha, beta, gamma = state[3:6]

        x_d, y_d, z_d = state[10:13]
        alpha_d, beta_d, gamma_d = state[13:16]

        r_w = np.array([[1, 0, -self.s(beta)], [0, self.c(alpha), self.s(alpha) * self.c(beta)],
                        [0, -self.s(alpha), self.c(alpha) * self.c(beta)]])

        rot = np.array([[self.c(beta)*self.c(gamma), self.c(beta)*self.s(gamma), -self.s(beta)],
                        [-self.c(alpha)*self.s(gamma)+self.s(alpha)*self.s(beta)*self.c(gamma), self.c(alpha)*self.c(gamma)+self.s(alpha)*self.s(beta)*self.s(gamma), self.s(alpha)*self.c(beta)],
                        [self.s(alpha)*self.s(gamma)+self.c(alpha)*self.s(beta)*self.c(gamma), -self.s(alpha)*self.c(gamma)+self.c(alpha)*self.s(beta)*self.s(gamma), self.c(alpha)*self.c(beta)]])

        w_sc = r_w.dot(state[3:6])
        v_sc = rot.dot(state[0:3])

        return v_sc, w_sc

