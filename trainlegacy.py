import os
import gym
#import pickle
from space_robot.envs import *
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecEnv,SubprocVecEnv,DummyVecEnv,VecMonitor
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.utils import set_random_seed

log_path = os.path.join('Training','Logs', 'PPOmulti200k')
PPO_Path = os.path.join('Training', 'Saved Models', 'PPOmulti200k')
post_path = os.path.join('Training', 'Saved Models', 'pretrainPPOmulti200k')

def make_env(rank, seed=0):
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environments you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """
    def _init():
        env = SpaceRobotEnv()
        
        env.seed(seed + rank)
        return env
    set_random_seed(seed)
    return _init

if __name__ == '__main__':
    #env_id = "CartPole-v1"
    num_cpu = 4  # Number of processes to use
    # Create the vectorized environment
    env = SubprocVecEnv([make_env(i) for i in range(num_cpu)])
    env = VecMonitor(env)# - temp uncomment
    # Stable Baselines provides you with make_vec_env() helper
    # which does exactly the previous steps for you.
    # You can choose between `DummyVecEnv` (usually faster) and `SubprocVecEnv`
    # env = make_vec_env(env_id, n_envs=num_cpu, seed=0, vec_env_cls=SubprocVecEnv)

    #model = PPO('MlpPolicy', env, verbose=1)
    #model.learn(total_timesteps=25_000)

    #obs = env.reset()
    #for _ in range(1000):
        #action, _states = model.predict(obs)
        #obs, rewards, dones, info = env.step(action)
        #env.render()

#env = SpaceRobotEnv()
#env = DummyVecEnv([lambda: env]) #might not need now?


    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
    #model = PPO.load(PPO_Path, env=env) #-- use if want to load existing model

    model.learn(total_timesteps=200000)#, callback = TensorboardCallback()
    obs = env.reset()
    for _ in range(200000):
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        #env.render()
    model.save(post_path)
#pickle.dump(model, open('PPO_200K.pkl','wb'))
