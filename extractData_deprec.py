import os
import pickle
from space_robot.envs import *
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor#, VecNormalize
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.utils import get_latest_run_id

#log_path = os.path.join('Training','Logs', 'PPO')
PPO_Path = os.path.join('Training', 'Saved_Models', 'testPPO_200K')
env = SpaceRobotEnv()
env = DummyVecEnv([lambda: env])
env = VecMonitor(env)
#model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
model = PPO.load(PPO_Path, env=env) #-- use if want to load existing model
#rewards

log_path = os.path.join('Training','Logs', 'testPPO_200K')
PPO_Path = os.path.join('Training', 'Saved_Models', 'testPPO_200K')
log_name="PPO"
latest_run_id = get_latest_run_id(log_path,log_name)
custom_log_path = os.path.join(log_path, f"{log_name}_{latest_run_id +1}")
csv_logger = configure(custom_log_path, ["stdout","csv","tensorboard","log"])

header = ['Episode', 'Reward

obs = env.reset()
#env.initialise_render() # needed if rendering is wanted --still broken need to fix somehow
episodes = 100
for episode in range (1, episodes+1):
	obs = env.reset()
	done = False
	score = 0
	while not done:
		#env.render() 
		action, _ = model.predict(obs)
		obs, reward, done, captured = env.step(action)
		score += reward
	print('Episode:{} Score:{}'.format(episode,score))
