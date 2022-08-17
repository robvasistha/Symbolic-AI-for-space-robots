from space_robot.envs import *

env = SpaceRobotEnv() # can use env = SpaceRobotEnv(dof=5) to change dof - default is 4
obs = env.reset()
env.initialise_render() # needed if rendering is wanted
done = False
while not done:
        action = env.action_space.sample()
        new_obs, reward, done, captured = env.step(action)
        print(reward)
        env.render()
