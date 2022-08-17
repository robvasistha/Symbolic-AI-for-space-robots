import numpy as np


class Mission():
    def __init__(self, dof):
        self.reward = 0
        self.dof = dof
        self.old_dist = 0.2

    def joint_limit(self, part_state, reward):
        ep = np.any((part_state[6:6+self.dof]) > 0.5)
        if ep == True:
            self.ep_done = True
            reward += -10
            # print('Joint limits exceeded at ', part_state[6:10], 'Reward is', reward)
        else:
            ep = np.any((part_state[6:6+self.dof]) < -0.5)
            if ep == True:
                self.ep_done = True
                reward += -10
                # print('Joint limits exceeded at ', part_state[6:10], 'Reward is', reward)
        return self.ep_done, reward

    def collision(self, done, complete, state):
        base_vel = np.linalg.norm(state[6+self.dof:12+self.dof])
        vel = state[6+self.dof:12+self.dof]
        # rr = np.subtract(self.inverse_kinematics(state, links, ff), np.array([[payload_loc[0]], [payload_loc[1]], [payload_loc[2]]]))
        # rr = self.inverse_kinematics(state, links, ff)
        # #print(np.linalg.norm(rr))
        capture = 0
        reward = 0
        if done == True:
            if complete == True:
                if base_vel < 0.1:
                    reward = 100
                    capture = 1
                    print('Payload Captured with velocity of less than', base_vel, 'Reward is', reward)
                else:
                    reward = 40
                    print('Payload Captured with base velocity of more than', base_vel, 'Reward is', reward)
                    capture = 1

            if complete == False:
                    reward = -50
                    #print('Spacecraft Collision', 'Reward is', reward)
        return reward, capture

    def reward_value(self, state, done, complete):
        ep = False
        re, capture = self.collision(done, complete, state)
        joint_lim, R = self.joint_limit(state, re)

        if done == True:
            ep = True

        if joint_lim == True:
            ep = True

        if complete == True:
            ep = True

        R -= sum(np.abs(state[6+self.dof:12+self.dof]))
        return R, ep, capture

    def reset(self):
        reward = 0
        self.ep_done = False
