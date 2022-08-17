import os
import pickle
from space_robot.envs import *
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv,VecMonitor
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy

log_path = os.path.join('Training','Logs', 'pretrainPPOmulti200k')
PPO_Path = os.path.join('Training', 'Saved Models', 'PPOmulti200k')
post_path = os.path.join('Training', 'Saved Models', 'pretrainPPOmulti200k')
env = SpaceRobotEnv()
env = DummyVecEnv([lambda: env]) #might not need now?
env = VecMonitor(env)

model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
model = PPO.load(PPO_Path, env=env)
#might need to adapt to pickle models.
model.learn(total_timesteps=200000)#, callback = TensorboardCallback()
model.save(PPO_Path)
