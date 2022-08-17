import os
import gym

from space_robot.envs import *
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv,VecMonitor
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.utils import get_latest_run_id

log_path = os.path.join('Training','Logs', 'PPO200K')
PPO_Path = os.path.join('Training', 'Saved_Models', 'PPO200K')
log_name="PPO"
latest_run_id = get_latest_run_id(log_path,log_name)
custom_log_path = os.path.join(log_path, f"{log_name}_{latest_run_id +1}")
csv_logger = configure(custom_log_path, ["stdout","csv","tensorboard","log"])

env = SpaceRobotEnv()
env = DummyVecEnv([lambda: env]) 
env = VecMonitor(env)

model = PPO('MlpPolicy', env, n_steps = 2048, verbose=1, tensorboard_log =log_path)
#model = PPO.load(PPO_Path, env=env)# -- use if want to load existing model
model.set_logger(csv_logger)
model.learn(total_timesteps=204800)
model.save(PPO_Path)

