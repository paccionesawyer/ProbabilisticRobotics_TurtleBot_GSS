from crowd_sim.envs.utils.agent import Agent
from crowd_sim.envs.utils.state import JointState, JointState_noV
import numpy as np

from scipy.linalg import block_diag

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class Robot(Agent):
    def __init__(self, config, section):
        super().__init__(config,section)
        # Added fields to track believed state for noisy implementations
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
        # Set believed state to true state initially
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
        # Create Kalman Filter
        self.kf = self.kalman(px, py, vx, vy)
    
    # Added function to initialize Kalman Filter
    def kalman(self, px, py, vx, vy) -> KalmanFilter:
        f = KalmanFilter(dim_x=4, dim_z=2, dim_u=2) # State 4x1, measurements 2x1, and control 2x1
        
        # Define KF state
        f.x = np.array([px, vx, py,  vy])
        
        # State transition matrix
        f.F = np.array([[1, self.time_step, 0,  0],
                        [0,  1, 0,  0],
                        [0,  0, 1, self.time_step],
                        [0,  0, 0,  1]])
        # Process noise
        q = Q_discrete_white_noise(dim=2, dt=self.time_step, var=self.noise_magnitude ** 2)
        f.Q = block_diag(q, q)
        
        # Format of control vector must be difference between action and previous velocity
        # u = np.array((action.vx - self.vx), (action.vy - self.vy))
        
        # Control transition matrix
        f.B = np.array([[0, 1, 0, 0],
                       [0, 0, 0, 1]]).T
        # Measurement function
        f.H = np.array([[1, 0, 0, 0],
                        [0, 0, 1, 0]])
        # Measurement noise
        f.R = np.array([[self.noise_magnitude ** 2., 0],
                        [0, self.noise_magnitude ** 2]])
        # Covariance matrix
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
    
    # Added function to access believed state for noisy implementation
    def get_noisy_full_state_noV(self):
        return [self.px_bel, self.py_bel, self.radius, self.gx, self.gy, self.v_pref, self.theta]

    def step(self, action):
        """
        Perform an action and update the state
        """
        self.check_validity(action)

        pos = self.compute_position(action, self.time_step)
        self.px, self.py = pos
        
        # Added variations of robot computation for each model variation
        if (self.mode == "normal"):
            self.px_bel, self.py_bel = pos
        elif (self.mode == "noisy"):
            pos_bel = self.compute_position_noise(action, self.time_step)
            self.px_bel, self.py_bel = pos_bel
        elif (self.mode == "kalman"):
            state = self.compute_position_kf(action, self.time_step)
            self.px_bel, self.vx_bel, self.py_bel, self.vy_bel = state

        if self.kinematics == 'holonomic':
            self.vx = action.vx
            self.vy = action.vy
            # print("Actual state: ", [self.px, self.vx, self.py, self.vy])
            # print()
    
    # Added function to compute noise in position
    def compute_position_noise(self, action, delta_t):
        self.check_validity(action)

        self.vx_bel = action.vx + np.random.normal(0, self.noise_magnitude)
        self.vy_bel = action.vy + np.random.normal(0, self.noise_magnitude)

        if self.kinematics == 'holonomic':
            px = self.px_bel + self.vx_bel * delta_t# + np.random.normal(0, 0.1*self.v_pref)
            py = self.py_bel + self.vy_bel * delta_t# + np.random.normal(0, 0.1*self.v_pref)


        return px, py
     
    # Added function to compute position using belief of Kalman Filter
    def compute_position_kf(self, action, delta_t):
        self.check_validity(action)

        u = np.array([(action.vx - self.vx_bel), (action.vy - self.vy_bel)]).T

        # self.vx_bel = action.vx + np.random.normal(0, self.noise_magnitude)
        # self.vy_bel = action.vy + np.random.normal(0, self.noise_magnitude)

        self.kf.predict(u=u)
        # print("Predicted state: ", self.kf.x)

        z = np.array([self.px + np.random.normal(0, self.noise_magnitude), self.py + np.random.normal(0, self.noise_magnitude)])
        self.kf.update(z)

        # print("Measured state: ", self.kf.x)

        if self.kinematics == 'holonomic':
            px = self.kf.x[0]
            vx = self.kf.x[1]
            py = self.kf.x[2]
            vy = self.kf.x[3]


        return px, vx, py, vy
