from Environment import *
from Space_Robot import *
import numpy as np
import torch
import torch.optim as optim
from torch.autograd import Variable
import torch.nn as nn
import torch.nn.functional as F
import gym
import random
from torch.distributions import Categorical
import matplotlib.pyplot as plt

class policy_network(nn.Module):
    def __init__(self):
        nn.Module.__init__(self)

        self.l1 = nn.Linear(52, 256)
        self.l2 = nn.Linear(256, 768)
        self.l3 = nn.Linear(768, 256)
        self.l4 = nn.Linear(256, 10)

    def forward(self, x):
        # forward function defines how the model will run
        x = F.relu(self.l1(x))
        x = F.relu(self.l2(x))
        x = F.relu(self.l3(x))
        x = F.softmax(self.l4(x), dim = -1)
        return (x)

class agent():
    def __init__ (self, gamma, lr_c):
        self.gamma = gamma
        self.learning_rate_critic = lr_c
        self.S = []
        self.R = []
        self.A = []
        self.N_S = []
        self.episode_reward = []
        self.episode_states = []
        self.episode_actions = []
        self.episode_next_state = []
        self.episode_memory = []

    def action(self, network, state):
        # probability of taking either action from that state is given by the network
        state = Variable(torch.FloatTensor(state))
        action = network(state)
        return action

    def clear_memory(self):
        self.R = []
        self.S = []
        self.A = []
        self.N_S = []
        self.episode_reward = []
        self.episode_actions = []
        self.episode_next_state = []
        self.episode_states = []

    def reward_value(self, T):
        # calcutes the total return from that episode
        R = np.zeros((T))
        reward = np.zeros((T))
        for t in range(T):
            r = self.R[t] * (self.gamma ** t)
            R[t] = r

        reward_mean = np.mean(R)
        reward_std = np.std(R)

        for i in range(T):
            reward[i] = (reward[i] - reward_mean) / reward_std
        return reward

    def store_step(self, state, action, reward, next_state):
        self.S.append(state)
        self.A.append(action)
        self.R.append(reward)
        self.N_S.append(next_state)

    def store_episode(self):
        self.episode_reward.append(torch.FloatTensor(self.R))
        self.episode_states.append(torch.FloatTensor(self.S))
        for A in self.A:
            self.episode_actions.append(torch.FloatTensor(A))
        self.episode_next_state.append(torch.FloatTensor(self.N_S))
        self.episode_memory = [self.episode_states, self.episode_actions, self.episode_reward, self.episode_next_state]
        return self.episode_memory

    def learn(self, networkA, optimizer):
        # implements gradient policy ascent
        log = torch.log(networkA(self.episode_states[0]))
        logprob = (networkC(self.episode_states[0]).detach().max(1)[0])*-log[np.arange(len(self.episode_actions)),self.episode_actions]
        loss = logprob.mean()

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        return loss

gamma = 0.9

space_robot = Space_Robot()

ff = np.array([0.3, 0.3, 0.3, 100])
l = np.array([[0, 0, 0], [0.2, 0, 0], [0.2, 0, 0], [0.1, 0, 0], [0.05, 0, 0]])
space_robot.set_parameters(l,ff)
env = Environment(space_robot)

learning_rate_actor = 0.0001

policy = policy_network()

optimizer = optim.RMSprop(policy.parameters(), lr=learning_rate_actor)

agent = agent(gamma, learning_rate_actor)
score = []
Episodes_completed = 0
episodes = 25

for e in range(episodes):
    state = env.reset()
    T = 0

    for f in range(100):
        action = agent.action(policy, state)
        action_np = action.detach().numpy()

        next_state, reward, done = env.step(action_np)
        agent.store_step(state, action, reward, next_state)
        st = next_state
        #env.render()
        if done == True:
            break
        T = +1

    score.append(T)
    expected_return = agent.reward_value(T)
    memory = agent.store_episode()
    l, logprob = agent.learn(policy, optimizer)
    agent.clear_memory()
    print('score = {}, Episode {}'.format(T, Episodes_completed))