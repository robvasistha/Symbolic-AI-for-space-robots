import os
import csv
from space_robot.envs import *
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv,VecMonitor
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy

log_path = os.path.join('Training','Logs', 'PPO_100K', 'PPOcsv.csv')
PPO_Path = os.path.join('Training', 'Saved_Models', 'PPO_100K')
#custom_log_path = os.path.join(log_path, f"PPO_{latest_run_id +1}")
env = SpaceRobotEnv()
env = DummyVecEnv([lambda: env])
env = VecMonitor(env)
#model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
model = PPO.load(PPO_Path, env=env) #-- use if want to load existing model
#rewards

header = ['Episode', 'Reward', 'Length']
ep_id = list(map(str,(range(1,101))))
#print (ep_id)

ep_rew, ep_len = evaluate_policy(model,env,n_eval_episodes=100,render=False,return_episode_rewards=True) 
#print("Avg reward over number of episodes: {} Deviation: {}".format(avg_reward,deviation))

with open(log_path, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(header)
    for item in zip(ep_id,ep_rew,ep_len):
        writer.writerow(item)
env.close()


