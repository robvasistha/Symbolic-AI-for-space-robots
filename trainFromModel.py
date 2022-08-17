import os
import pickle
from space_robot.envs import *
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.utils import get_latest_run_id

env = SpaceRobotEnv()
env = DummyVecEnv([lambda: env])
env = VecMonitor(env) # can use env = SpaceRobotEnv(dof=5) to change dof - default is 4


log_path = os.path.join('Training','Logs', 'PPO_100K')
PPO_Path = os.path.join('Training', 'Saved_Models', 'PPO_100K')
new_PPO_Path = os.path.join('Training', 'Saved_Models', 'PPO_100Ktrained')
final_PPO_Path = os.path.join('Training', 'Saved_Models', 'PPO_100Ktrainedfinal')
log_name="PPO"
latest_run_id = get_latest_run_id(log_path,log_name)
custom_log_path = os.path.join(log_path, f"{log_name}_{latest_run_id +1}")
csv_logger = configure(custom_log_path, ["stdout","csv","tensorboard","log"])

#model = PPO('MlpPolicy', env, n_steps = 2048, verbose=1)
model = PPO.load(PPO_Path, n_steps = 64,env=env)
model.set_logger(csv_logger)
model.learn(total_timesteps=102400)#,callback = 
obs = env.reset()

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
env.close()


model.save(PPO_Path)
del model

model = PPO.load(PPO_Path, n_steps = 2048, env=env)
model.set_logger(csv_logger)
model.learn(total_timesteps = 64000)
model.save(new_PPO_Path)
del model

model = PPO.load(new_PPO_Path, n_steps = 2048, env=env)
model.set_logger(csv_logger)
model.learn(total_timesteps = 64000)
model.save(final_PPO_Path)


