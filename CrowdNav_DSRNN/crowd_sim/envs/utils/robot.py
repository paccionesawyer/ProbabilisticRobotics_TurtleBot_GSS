from crowd_sim.envs.utils.agent import Agent
from crowd_sim.envs.utils.state import JointState, JointState_noV

class Robot(Agent):
    def __init__(self, config, section):
        super().__init__(config,section)
        self.px_bel = None
        self.py_bel = None
        self.vx_bel = None
        self.vy_bel = None
        self.noise_magnitude = config.noise_magnitude

    def act(self, ob):
        if self.policy is None:
            raise AttributeError('Policy attribute has to be set!')

        state = JointState(self.get_full_state(), ob)
        action = self.policy.predict(state)
        return action

    def act_noV(self, ob):
        if self.policy is None:
            raise AttributeError('Policy attribute has to be set!')
        state = JointState_noV(self.get_full_state(), ob)
        action = self.policy.predict(state)
        return action

    def actWithJointState(self,ob):
        action = self.policy.predict(ob)
        return action

    def get_noisy_full_state_noV(self):
        return [self.px_bel, self.py_bel, self.radius, self.gx, self.gy, self.v_pref, self.theta]

    def step(self, action):
        """
        Perform an action and update the state
        """
        self.check_validity(action)
        pos = self.compute_position(action, self.time_step)
        self.px, self.py = pos
        self.px_bel, self.py_bel = pos + np.random.normal(0, self.noise_magnitude, size=2)
        if self.kinematics == 'holonomic':
            self.vx = action.vx
            self.vy = action.vy

            self.vx_bel = action.vx + np.random.normal(0, self.noise_magnitude)
            self.vy_bel = action.vy + np.random.normal(0, self.noise_magnitude)
        else:
            self.theta = (self.theta + action.r) % (2 * np.pi)
            self.vx = action.v * np.cos(self.theta)
            self.vy = action.v * np.sin(self.theta)
