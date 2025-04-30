import numpy as np
import yaml
from os.path import getmtime, isfile
from enum import Enum
from time import time


class BatteryModelState:
    """
    General battery model marams
    """

    R1: float
    C1: float
    R:  float
    Q:  float

    def __init__(self) -> None:
        self.R1 = 0.3
        self.C1 = 1012.0
        self.R  = 0.035
        self.Q  = 16.0

    def __call__(self, *args, **kwds):
        return np.array([[self.R1], [self.C1], [self.R], [self.Q]])


    def from_array(self, arr: np.ndarray) -> None:
        """
        Fill state form np array

        :params arr: state vactor [R1, C1, R, Q].T
        """
        self.R1 = max(0.00001, arr[0][0])
        self.C1 = max(0.00001, arr[1][0])
        self.R  = max(0.00001, arr[2][0])
        self.Q  = max(0.00001, arr[3][0])



class State:
    """
    General battery state model 
    """
    SoC:  float
    V1:  float

    def __init__(self) -> None:
        self.SoC = 0
        self.V1  = 0

    def __call__(self, *args, **kwds) -> np.ndarray:
        return np.array([[self.SoC], [self.V1]])

    def from_array(self, arr: np.ndarray) -> None:
        """
        Fill state form np array

        :params arr: state vactor [SoC, V1].T
        """
        self.SoC = max(0.0, min(1.0, arr[0][0])) 
        self.V1  = max(0.0, arr[1][0])


class Params:
    """
    General battery params
    """

    # Battery parameters
    R0: float        # Battery internal resistance   [in Ohm]
    Q0: float        # Battery rated capacity        [in Ah]

    Vmax: float      # Full battery voltage          [in V]
    Vmin: float      # Empty battery voltage         [in V]

    # Identity matrices
    I_4:    np.ndarray = np.diag([1, 1, 1, 1])
    I_2:    np.ndarray = np.diag([1, 1])

    # Covariance variables
    F: np.ndarray    # For the model state equation
    G: float         # For the output relation

    D: np.ndarray    # For the rq-model state equation
    E: float         # For the internal resistance

    # Battery state linear system martices [ A(O)*x(k) + B(O)*u(k) ]

    def A(self, dt, param: BatteryModelState) -> np.ndarray:
        return np.diag([1, np.e ** ( -dt / (param.R1 * param.C1))])


    def B(self, dt, param: BatteryModelState) -> np.ndarray:
        return np.array([[- dt / (param.Q * 3600)],
                         [param.R1 * ( 1 - np.e ** ( -dt / (param.R1 * param.C1)))]])
    
    # Partial derivative matrices for param covariance calculations

    def dx(self, dt, param: BatteryModelState, u):
        return np.diag([1, np.e ** ( -dt / (param.R1 * param.C1))])

    def dO(self, dt, state: State, param: BatteryModelState, u) -> np.ndarray:
        return np.array([[0, 0, 0, u * dt /  (param.Q * 3600 )**2],
                         [
                            u + ( np.e ** ( -dt / (param.R1 * param.C1)) / param.R1 ) * ( state.V1 / (param.R1 * param.C1) - u * param.R1 + u * dt /param.C1 ),
                            (dt * np.e ** ( -dt / (param.R1 * param.C1)) / param.C1 ** 2) * (state.V1 / param.R1 - u),
                            0, 
                            0
                         ]])


    def init_cov_matrices(self) -> None:
        """
        Init covariance matrices
        """
        self.G =  1 / 10**4
        self.F = np.diag([1 / 10**2,
                          1 / 10**5,])
        # self.G = 0.8

        self.E =  1 / 10**4

        self.D = np.diag([1 / 10**6,
                          1 / 10**1,
                          1 / 10**6,
                          1 / 10**2,])

    def init(self, R0, Q0, Vmax, Vmin) -> None:
        """
        Initialise parameters by hand

        :params R0:   Battery internal resistance   [in Ohm]
        :params Q0:   Battery rated capacity        [in Ah]
        :params Vmax: Full battery voltage          [in V]
        :params Vmin: Empty battery voltage         [in V]
        """

        self.R0 = R0
        self.Q0 = Q0
        self.Vmax = Vmax
        self.Vmin = Vmin

        self.init_cov_matrices()


    def init_from_file(self, path) -> bool:
        """
        Initialise parameters from file

        :params path: path to config file
        """
        if not isfile(path):
            return False

        try:
            with open(path) as f:
                cfg = yaml.load(f, Loader=yaml.FullLoader)

                self.R0   = cfg["battery"]["R0"]
                self.Q0   = cfg["battery"]["Q0"]
                self.Vmax = cfg["battery"]["Vmax"]
                self.Vmin = cfg["battery"]["Vmin"]
        except Exception:
            return False

        self.init_cov_matrices()

        return True


class StateParams:
    """
    Saved battery params
    """

    R: float        # Battery internal resistance   [in Ohm]
    Q: float        # Battery rated capacity        [in Ah]

    R1: float       # Battery charge transfer resistance [in Ohm]
    C1: float       # Battery charge transfer capacity   [in F]

    threshold: int  # Time delta, when data in the file is still valid 

    class FileState(Enum):
        """
        Config file state enum
        """

        EXISTS_VALID   = 0
        EXISTS_INVALID = 1
        NONEXISTANT    = 2

    def __init__(self, threshold) -> None:
        self.threshold = threshold

    def init_from_file(self, path) -> True:
        """
        Initialise parameters from file
        
        :params path: path to config file
        """
        if not isfile(path):
            return False

        try:
            with open(path) as f:
                cfg = yaml.load(f, Loader=yaml.FullLoader)

                self.R   = cfg["battery"]["R"]
                self.Q   = cfg["battery"]["Q"]
                self.R1  = cfg["battery"]["R1"]
                self.C1  = cfg["battery"]["C1"]
        except Exception:
            return False

        return True

    def check_file(self, path) -> FileState:
        """
        Initialise parameters from file
        
        :params path: path to config file
        """
        if not isfile(path):
            return self.FileState.NONEXISTANT

        if time() - getmtime(path) > self.threshold:
            return self.FileState.EXISTS_INVALID
        
        return self.FileState.EXISTS_VALID


    def write_to_file(self, path) -> None:
        """
        Write parameters to config file
        
        :params path: path to config file
        """
        mod = "w"
        if not isfile(path):
            mod = "x"

        with open(path, mod) as f:
            yaml.dump({"battery": {"R": self.R, "Q": self.Q, "R1": self.R1, "C1": self.C1}}, f)
