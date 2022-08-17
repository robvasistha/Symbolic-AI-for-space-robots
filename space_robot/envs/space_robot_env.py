from space_robot.envs.back.Dynamics_four import *
from space_robot.envs.back.Dynamics_five import *
from space_robot.envs.back.Dynamics_six import *
from space_robot.envs.back.Payload import *
from space_robot.envs.back.Render import *
from space_robot.envs.back.Collision import *
from space_robot.envs.back.Mission import *
from space_robot.envs.back.Space_Robot import *
from space_robot.envs.back.ref_frame import *
import gym
from gym import spaces
from gym.utils import seeding

class SpaceRobotEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self, dof=None):
        #Default dof is 4
        if dof == None:
            self.dof = 4
        else:
            self.dof = dof
        self.space_robot = Space_Robot(self.dof)
        if self.dof == 4:
            self.dynamics = Dynamics_four()
        if self.dof == 5:
            self.dynamics = Dynamics_five()
        if self.dof == 6:
            self.dynamics = Dynamics_six()

        self.collision = Collision(self.dof)
        self.mission = Mission(self.dof)
        self.payload = Payload()
        self.space_robot.set_parameters()
        self.ref_frame = 'world'
        self.reward = 0
        # self.change_frame = change_frame()
        lim = []
        #
        #   Observation space: base position [0:6]
        #                      arm angles [6:6+self.dof]
        #                      base velocity [6+self.dof:12+self.dof]
        #                      arm velocity [12+self.dof:12+(self.dof*2)]
        #                      payload loc  [12+(self.dof*2):18+(self.dof*2)]
        #                      payload size [18+(self.dof*2):21+(self.dof*2)]
        #                      payload mass [21+(self.dof*2):22+(self.dof*2)]

        for i in range(22 + (self.dof*2)):
            lim.append(10)
        lim = np.array(lim)

        self.observation_space = spaces.Box(-lim, lim, dtype=np.float32)
        self.action_space = spaces.Box(low=-0.05,
                                       high=0.05, shape=(6+self.dof,))
        self.t = 0

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def final_position_error(self):
        try:
            c = self.collision.PayloadNode.getPos(self.collision.dummynode) - self.collision.eenode.getPos(self.collision.dummynode)
        except:
            c = np.nan
        return c

    def step(self, action):
        # state = position, velocity, form factor and mass, link lengths, payload location, payload size, finished
        action[0:3] = action[0:3]
        action[3:6] = action[3:6]/10000
        action[6:6+self.dof] = action[6:6+self.dof]/1000000
        self.t += 1

        self.space_robot_state = self.dynamics.space_robot_step(action, self.space_robot_state)


        done, complete = self.collision.check_collision(self.space_robot_state, self.space_robot.get_base_dimensions(),
                                                        self.space_robot.get_link_lengths(),
                                                        self.payload.get_size(),
                                                        self.payload.get_location())

        self.reward, finished, capture = self.mission.reward_value(self.space_robot_state, done, complete)

        self.new_state = np.concatenate((self.space_robot_state, self.payload_loc, self. payload_size,
                                        np.array([self.payload_mass])))

        self.capture = capture

        return self.new_state, self.reward, finished, {}

    def initialise_render(self):
        self.world = Render(self.dof)
        self.capture = 0
        self.world.render(self.space_robot_state, self.space_robot.get_base_dimensions(),
                          self.space_robot.get_link_lengths(),
                          self.payload.get_location(), self.payload.get_size(), self.capture)

    def HER_reward(self, state):
        self.dynamics.reset(self.space_robot)
        self.collision.reset()
        self.mission.reset()

        # State sorted in function state is not updated with HER eps as these should not be shown to the dynamics.
        st = state[0:12+(self.dof*2)]
        payload_pos = state[12+(self.dof*2):18+(self.dof*2)]
        payload_size = state[18+(self.dof*2):21+(self.dof*2)]

        done, complete = self.collision.check_collision(st, self.space_robot.get_base_dimensions(),
                                                        self.space_robot.get_link_lengths(),
                                                        payload_size,
                                                        payload_pos)

        reward, finished, capture = self.mission.reward_value(st, done, complete)


        return reward, finished, capture

    def render_reset(self):
        self.world.reset()

    def get_link_lengths(self):
        return self.space_robot.get_link_lengths()

    def render(self,mode='train'):
        if self.ref_frame == 'world':
            self.world.render(self.space_robot_state, self.space_robot.get_base_dimensions(),
                              self.space_robot.get_link_lengths(),
                              self.payload.get_location(), self.payload.get_size(), self.capture)
        else:
            self.world.render(self.new_state[0:20], self.state.get_base_dimensions(),
                                self.state.get_link_length(),
                                self.new_state[20:26], self.state.get_payload_size())

    def show(self):
        if self.ref_frame == 'world':
            self.world.render(self.space_robot_state, self.space_robot.get_base_dimensions(),
                              self.space_robot.get_link_lengths(),
                              self.payload.get_location(), self.payload.get_size(), self.capture)
        else:
            self.world.render(self.new_state[0:20], self.state.get_base_dimensions(),
                                self.state.get_link_length(),
                                self.new_state[20:26], self.state.get_payload_size())

    def update_payload(self, payload):
        self.payload = payload.set_location(payload)

    def update_scale(self, l):
        self.space_robot.set_linklengths(l)

    def get_link_length(self):
        return self.space_robot.get_link_lengths()

    def reset(self):
        self.space_robot_state = self.dynamics.reset(self.space_robot)
        self.payload_size, self.payload_loc, self.payload_mass = self.payload.generate_random()
        self.new_state = np.concatenate((self.space_robot_state, self.payload_loc, self. payload_size,
                                        np.array([self.payload_mass])))
        # self.payload.set_location(np.array([-0.3, 0.2, 1.2, 0, 0, 0]))
        # self.payload.set_size(np.array([0.2, 0.2, 0.2]))
        self.collision.reset()
        self.t = 0
        self.mission.reset()
        return self.new_state
