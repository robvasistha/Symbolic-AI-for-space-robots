import os
from space_robot.envs import *
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import VecMonitor
from stable_baselines3.common.evaluation import evaluate_policy

env = SpaceRobotEnv() # can use env = SpaceRobotEnv(dof=5) to change dof - default is 4
obs = env.reset()
env.initialise_render() # needed if rendering is wanted --still broken need to fix somehow
episodes = 100
for episode in range (1, episodes+1):
	obs = env.reset()
	done = False
	score = 0
	while not done:
		env.render() 
		action = env.action_space.sample()
		new_obs, reward, done, captured = env.step(action)
		score += reward
	print('Episode:{} Score:{}'.format(episode,score))
#env.close()
