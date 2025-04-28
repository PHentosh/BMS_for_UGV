import numpy as np
from numpy.linalg import inv
from battery_monitor.bms_utils import State, BatteryModelState
from battery_monitor.battery_model import BatterySoCModel, BatteryParamModel


class EkfSoC:
    """
    Extended Kalman filter class for determining battery state of change (SoC)
    """

    last_state:    State
    last_cov:      np.ndarray

    battery:       BatterySoCModel

    def __init__(self, battery: BatterySoCModel) -> None:
        self.battery = battery

    def init(self, x: State) -> None:
        """
        Initialise parameters for extended Kalman filter

        :param x:             initail system state
        :param i_in:          initial system input
        :param battery_state: additional system parmeters
        """

        self.last_state    = x
        self.last_cov      = self.battery.init_state_covariance()


    def state_update(self, in_i: float, params: BatteryModelState, dt: float):
        """
        Make state vector prediction

        :param in_i:    measured system input
        :param params:  battery model state parameters
        :param dt:      time step in seconds

        :returns: state vector
        """

        # Get noise
        w = np.random.normal(0, self.battery.params.F)

        # Predict rough state
        self.last_state.from_array(self.battery.state(self.last_state, in_i, params, np.array([w.diagonal().tolist()]).T, dt))

        return self.last_state


    def state_innovation(self, in_i: float, in_v: float, params: BatteryModelState, dt: float):
        """
        Make state vector correction

        :param in_i:    measured system input
        :param in_v:    measured system output
        :param params:  battery model state parameters
        :param dt:      time step in seconds

        :returns: predicted system output
        """

        # Get noise
        v = np.random.normal(0, self.battery.params.G)

        # Get covariance for prediction
        P     =  (self.battery.params.A(dt, params) @ self.last_cov @ self.battery.params.A(dt, params).T) + self.battery.params.F

        # Predict model output and it's covariance
        new_V = self.battery.output(self.last_state, params, in_i, v)
        C     = self.battery.output_jacobian(self.last_state)
    
        print("new V soc: ", new_V)
        print("diff soc:  ", in_v - new_V)
        print("pre gues:\n", self.last_state())
        
        # Kalman gain
        L = P @ C.T @ inv(C @ P @ C.T + self.battery.params.G) 

        print("L:\n", L)
        # print("P:\n", P)
        # print("C:\n", C)

        # Update variables
        self.last_state.from_array(self.last_state() + L * (in_v - new_V))
        self.last_cov  = (self.battery.params.I_2 - L @ C) @ P

        return new_V



    @property
    def state(self) -> State:
        return self.last_state

    @property
    def covariance(self) -> np.ndarray:
        return self.last_cov



class EkfParams:
    """
    Extended Kalman filter class for determining battery internal resistance (R) and capacity (Q)
    """

    last_state: BatteryModelState
    last_cov:   np.ndarray
    last_jacob: np.ndarray

    battery: BatteryParamModel

    def __init__(self, battery: BatteryParamModel) -> None:
        self.battery = battery

    def init(self, x: BatteryModelState) -> None:
        """
        Initialise parameters for extended Kalman filter

        :param x:             initail system state
        :param i_in:          initial system input
        :param battery_state: additional system parmeters
        """
    
        self.last_state    = x
        self.last_cov      = self.battery.init_state_covariance()
        self.last_jacob    = self.battery.init_partial_output_jacobian()


    def state_update(self):
        """
        Make parameters state vector prediction

        :returns: state vector
        """

        # Get noise
        w = np.random.normal(0, self.battery.params.D)
        
        # Predict rough state
        self.last_state.from_array(self.battery.param_state(self.last_state, np.array([w.diagonal().tolist()]).T))

        return self.last_state


    def state_innovation(self, in_i: float, in_v: float, battery_state: State, dt):
        """
        Make parameters state vector correction

        :param in_i:           measured system input
        :param in_v:           measured system output
        :param battery_state:  battery model state vector
        :param dt:             time step in seconds

        :returns: predicted system output
        """

        # Get noise
        v = np.random.normal(0, self.battery.params.E)

        # Get covariance for prediction
        P = self.last_cov + self.battery.params.D

        # Predict model output and it's covariance
        new_V = self.battery.output(battery_state, self.last_state, in_i, v)
        C, self.last_jacob  = self.battery.param_output_jacobian(battery_state, self.last_state, self.last_jacob, in_i, dt)
        
        # Kalman gain
        L = P @ C.T @ inv(C @ P @ C.T + self.battery.params.E)

        # Update variables
        self.last_state.from_array(self.last_state() + L * (in_v - new_V))
        self.last_cov = (self.battery.params.I_4 - L @ C) @ P

        return new_V


    
    @property
    def state(self) -> BatteryModelState:
        return self.last_state

    @property
    def covariance(self) -> np.ndarray:
        return self.last_cov
    
