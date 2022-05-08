from crowd_sim.envs.utils.agent import Agent
from crowd_sim.envs.utils.state import JointState, JointState_noV
import numpy as np

from scipy.linalg import block_diag

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class Robot(Agent):
    def __init__(self, config, section):
        super().__init__(config,section)
        self.px_bel = None
        self.py_bel = None
        self.vx_bel = None
        self.vy_bel = None
        self.noise_magnitude = config.noise.magnitude
        self.mode = config.robot.mode

    def act(self, ob):
        if self.policy is None:
            raise AttributeError('Policy attribute has to be set!')

        state = JointState(self.get_full_state(), ob)
        action = self.policy.predict(state)
        return action

    def set(self, px, py, gx, gy, vx, vy, theta, radius=None, v_pref=None):
        self.px = px
        self.py = py
        self.px_bel = px # initialize believed state to true state
        self.py_bel = py
        self.gx = gx
        self.gy = gy
        self.vx = vx
        self.vy = vy
        self.vx_bel = vx
        self.vy_bel = vy
        self.theta = theta

        if radius is not None:
            self.radius = radius
        if v_pref is not None:
            self.v_pref = v_pref

        self.kf = self.kalman(px, py, vx, vy)

    def kalman(self, px, py, vx, vy) -> KalmanFilter:
        f = KalmanFilter(dim_x=4, dim_z=2, dim_u=2)

        f.x = np.array([px, vx, py,  vy]).T 

        f.F = np.array([[1, self.time_step, 0,  0],
                        [0,  1, 0,  0],
                        [0,  0, 1, self.time_step],
                        [0,  0, 0,  1]])

        q = Q_discrete_white_noise(dim=2, dt=self.time_step, var=self.noise_magnitude ** 2)
        f.Q = block_diag(q, q)

        # u = np.array((action.vx - self.vx), (action.vy - self.vy))

        f.B = np.array([[0, 1, 0, 0],
                       [0, 0, 0, 1]])

        f.H = np.array([[1, 0, 0, 0],
                        [0, 0, 1, 0]])

        f.R = np.array([[self.noise_magnitude ** 2., 0],
                        [0, self.noise_magnitude ** 2]])

        f.P = np.eye(4) * self.noise_magnitude ** 2.

        return f

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
        self.px_bel, self.py_bel = pos

        if (self.mode == "noisy"):
            pos2 = self.compute_position_noise(action, self.time_step)
            self.px_bel, self.py_bel = pos2

        elif (self.mode == "kalman"):
            state = self.compute_position_kf(action, self.time_step)
            self.px_bel, self.vx_bel, self.py_bel, self.vy_bel = state

        if self.kinematics == 'holonomic':
            self.vx = action.vx
            self.vy = action.vy

    def compute_position_noise(self, action, delta_t):
        self.check_validity(action)

        self.vx_bel = action.vx + np.random.normal(0, self.noise_magnitude)
        self.vy_bel = action.vy + np.random.normal(0, self.noise_magnitude)

        if self.kinematics == 'holonomic':
            px = self.px_bel + self.vx_bel * delta_t# + np.random.normal(0, 0.1*self.v_pref)
            py = self.py_bel + self.vy_bel * delta_t# + np.random.normal(0, 0.1*self.v_pref)


        return px, py

    def compute_position_kf(self, action, delta_t):
        self.check_validity(action)

        u = np.array([(action.vx - self.vx_bel), (action.vy - self.vy_bel)])

        # self.vx_bel = action.vx + np.random.normal(0, self.noise_magnitude)
        # self.vy_bel = action.vy + np.random.normal(0, self.noise_magnitude)

        self.kf.predict(u=u)

        z = np.array([self.px + np.random.normal(0, self.noise_magnitude), self.py + np.random.normal(0, self.noise_magnitude)])
        self.kf.update(z)

        if self.kinematics == 'holonomic':
            px = self.kf.x[0]
            vx = self.kf.x[1]
            py = self.kf.x[2]
            vy = self.kf.x[3]


        return px, vx, py, vy
