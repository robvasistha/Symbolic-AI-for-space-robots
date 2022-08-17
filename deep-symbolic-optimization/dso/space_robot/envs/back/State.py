import numpy as np

class State():
    def __init__(self, space_robot, payload):
        self.base_linear_position = np.array([0,0,0])
        self.base_angular_position = np.array([0,0,0])
        self.base_linear_velocity = np.array([0,0,0])
        self.base_angular_velocity = np.array([0,0,0])
        self.theta = np.array([0,0,0,0])
        self.theta_d = np.array([0,0,0,0])
        self.link_length = space_robot.get_link_lengths_matrix()
        self.base_dimensions = space_robot.get_base_dimensions()
        self.base_mass = space_robot.get_base_mass()
        self.payload_size = payload.get_size()
        self.payload_position = payload.get_location()
        self.reward = 0
        self.ep_done = False
        self.part_state = np.concatenate(
            [self.base_linear_position, self.base_angular_position, self.theta, self.base_linear_velocity,
             self.base_angular_velocity, self.theta_d])
        full_state = np.concatenate(
            [self.base_linear_position.flatten(), self.base_angular_position.flatten(), self.theta.flatten(),
             self.base_linear_velocity.flatten(), self.base_angular_velocity.flatten(), self.theta_d.flatten(),
             self.payload_position, self.payload_size, self.link_length.flatten(), self.base_dimensions,
             self.base_mass.flatten()])
        self.full_state = np.array([full_state, self.reward, self.ep_done])

    def complie_part_state(self, ln, phi, theta, ln_d, phi_d, theta_d):
        self.base_linear_position = ln
        self.base_angular_position = phi
        self.theta = theta
        self.base_linear_velocity = ln_d
        self.base_angular_velocity = phi_d
        self.theta_d = theta_d
        self.part_state = np.concatenate(
            [self.base_linear_position.flatten(), self.base_angular_position.flatten(), self.theta.flatten(),
             self.base_linear_velocity.flatten(), self.base_angular_velocity.flatten(), self.theta_d.flatten()])
        self.compile_full_state()

    def update_part_state(self, state):
        self.base_linear_position = state[0:3].flatten()
        self.base_angular_position = state[3:6].flatten()
        self.theta = state[6:10].flatten()
        self.base_linear_velocity = state[10:13].flatten()
        self.base_angular_velocity = state[13:16].flatten()
        self.theta_d = state[16:20].flatten()
        self.part_state = np.concatenate(
            [self.base_linear_position, self.base_angular_position, self.theta,
             self.base_linear_velocity, self.base_angular_velocity, self.theta_d])

    def compile_base_linear_position(self, base_position):
        self.base_linear_position = base_position.flatten()
        self.compile_full_state()

    def compile_base_angular_position(self, base_position):
        self.base_angular_position = base_position.flatten()
        self.compile_full_state()

    def compile_base_linear_velocity(self, base_velocity):
        self.base_linear_velocity = base_velocity.flatten()
        self.compile_full_state()

    def compile_base_angular_velocity(self, base_velocity):
        self.base_angular_velocity = base_velocity.flatten()
        self.compile_full_state()

    def compile_theta(self, theta):
        self.theta = theta.flatten()
        self.compile_full_state()

    def compile_theta_d(self, theta_d):
        self.theta_d = theta_d.flatten()
        self.compile_full_state()

    def compile_payload_position(self, payload_position):
        self.payload_position = payload_position
        self.compile_full_state()

    def compile_payload_size(self, payload_size):
        self.payload_size = payload_size
        self.compile_full_state()

    def compile_link_length(self, l):
        if len(l) == 4:
            self.link_length = np.array([[0, 0, 0], [l[0], 0, 0], [l[1], 0, 0], [l[2], 0, 0], [l[3], 0, 0]])
        if len(l) == 5:
            self.link_length = np.array([[0, 0, 0], [l[0], 0, 0], [l[1], 0, 0], [l[2], 0, 0], [l[3], 0, 0], [l[4], 0, 0]])
        if len(l) == 6:
            self.link_length = np.array([[0, 0, 0], [l[0], 0, 0], [l[1], 0, 0], [l[2], 0, 0], [l[3], 0, 0], [l[4], 0, 0], [l[5], 0, 0]])

        self.compile_full_state()

    def compile_base_mass(self, base_mass):
        self.base_mass = base_mass
        self.compile_full_state()

    def compile_base_dimensions(self, base_dim):
        self.base_dimensions = base_dim
        self.compile_full_state()

    def compile_full_state(self):
        # full_state = np.concatenate(
        #     [self.base_linear_position, self.base_angular_position, self.theta,
        #      self.base_linear_velocity, self.base_angular_velocity, self.theta_d,
        #      self.payload_position, self.payload_size, self.link_length.flatten(), self.base_dimensions,
        #      self.base_mass.flatten()])
        full_state = np.concatenate(
            [self.base_linear_position, self.base_angular_position, self.theta,
             self.base_linear_velocity, self.base_angular_velocity, self.theta_d,
             self.payload_position, self.payload_size])

        self.part_state = np.concatenate(
            [self.base_linear_position, self.base_angular_position, self.theta,
             self.base_linear_velocity, self.base_angular_velocity, self.theta_d])
        self.full_state = full_state

    def get_part_state(self):
        self.compile_full_state()
        return self.part_state

    def get_base_angular_position(self):
        self.compile_full_state()
        return self.base_angular_position

    def get_base_linear_position(self):
        self.compile_full_state()
        return self.base_linear_position

    def get_base_linear_velocity(self):
        self.compile_full_state()
        return self.base_linear_velocity

    def get_base_angular_velocity(self):
        self.compile_full_state()
        return self.base_angular_velocity

    def get_theta(self):
        self.compile_full_state()
        return self.theta

    def get_theta_d(self):
        self.compile_full_state()
        return self.theta_d

    def get_payload_position(self):
        self.compile_full_state()
        return self.payload_position

    def get_payload_size(self):
        self.compile_full_state()
        return self.payload_size

    def get_link_length(self):
        self.compile_full_state()
        return self.link_length

    def get_base_mass(self):
        self.compile_full_state()
        return self.base_mass

    def get_base_dimensions(self):
        self.compile_full_state()
        return self.base_dimensions

    def get_reward(self):
        self.compile_full_state()
        return self.reward

    def update_episode(self, ep):
        self.ep_done = ep
        self.compile_full_state()

    def get_info(self):
        return self.ep_done

    def update_reward(self, reward):
        self.reward = reward
        self.compile_full_state()

    def get_full_state(self):
        self.compile_full_state()
        return self.full_state

