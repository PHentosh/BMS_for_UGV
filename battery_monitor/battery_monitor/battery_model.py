import numpy as np
from battery_monitor.bms_utils import Params, State, BatteryModelState


class BatteryModel:
    params: Params

    def __init__(self, params: Params):
        self.params = params

    def ocv(self, soc) -> float:
        """
        Open circuit voltage (OCV), function of SoC

        ## Warning
        It's linear model for now. Change to real OCV to SoC curve for more precision.

        :param soc: state of charge

        :returns: ocv value
        """
        return self.params.Vmin * (1 - soc) + self.params.Vmax * soc


    def ocv_jacobian(self, soc: float) -> float:
        """
        Jacobian (sensitivity) of OCV with respect to SoC

        ∂Voc/∂SoC

        ## Warning
        It's linear model for now. Change to real OCV to SoC function for more precision.

        :param soc: state of charge

        :returns: jacobian value
        """
        return (self.params.Vmax - self.params.Vmin)/1.0


    def output(self, x: State, params: BatteryModelState, u:float, v:float) -> float:
        """
        Battery output equation

        y(k) = g(x(k), u(k), v(k))

        V(k) = Vocv(SoC(k)) - V1(k) - V2(k) - R0 * I(k) + v(k)

        :param x: state vector
        :param u: measured system input
        :param v: white gaussian noise

        :returns: battery voltage
        """
        return self.ocv(x.SoC) - x.V1 - params.R * u + v


    def output_jacobian(self, x: State) -> np.ndarray:
        """
        First order derivative of battery output equation:

        J = ∂g/∂x at (x,u) = ∂(V(k))/∂(SoC(k))


        Battery output equation

        y(k) = g(x(k), u(k), v(k))

        V(k) = Vocv(SoC(k)) - Vct(k) - R0 * I(k) + v(k)


        :param x: state vector

        :returns: jacobian vector

        """
        return np.array([[self.ocv_jacobian(x.SoC), -1]])



class BatterySoCModel(BatteryModel):

    def __init__(self, params: Params):
        super().__init__(params)


    def state(self, x:State, u:float, p: BatteryModelState, w: np.ndarray, dt: float) -> np.ndarray:
        """
        Battery state dynamics equation

        x(k) = A*x(k-1) + B*u(k) + w(k)

        :param x:  state vector
        :param u:  measured system input
        :param p:  battery model parameters
        :param w:  white gaussian noise
        :param dt: time step in seconds

        :returns: state vector
        """

        return self.params.A(dt, p) @ x() + self.params.B(dt, p) * u + w


    def init_state_covariance(self) -> np.ndarray:
        """
        Get initial value for battery state covariance
        
        :returns: battery covariance
        """
        return np.diag([1, 1])



class BatteryParamModel(BatteryModel):

    def __init__(self, params: Params):
        super().__init__(params)


    def param_state(self, x: BatteryModelState, w: np.ndarray) -> np.ndarray:
        """
        Battery state dynamics equation

        Q(k) = Q(k-1) + w(k)

        :param q:  RQ state vector
        :param w:  white gaussian noise

        :returns: state vector
        """
        return x() + w


    def param_output_jacobian(self, x: State, p: BatteryModelState, D:  np.ndarray, u: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        First order derivative of battery output equation:

        J = ∂g/∂θ at (x,u,θ) = dg(x(k), u(k), θ(k)) / dθ

        dg (x, u, θ)/dQ = ∂g(x, u, Q)/∂θ  + ∂g(x, u, θ)/∂x  * dx/dθ

        Battery output equation

        y(k) = g(x(k), u(k), θ(k))

        V(k) = Vocv(SoC(k)) - V1(k) - R0 * I(k) + v(k)


        :param x: state vector
        :param p: battery model parameters
        :param D: partial derivative of dx/dθ form previous step
        :param u: measured system input 

        :returns: jacobian vector

        """
        D =  self.params.dO(dt, x, p, u) + self.params.dx(dt, p, u) @ D
        dg_do = np.array([[0, 0, -u, 0]])
        jacob = dg_do + self.output_jacobian(x) @ D

        return jacob, D
    
    
    def init_partial_output_jacobian(self) -> np.ndarray:
        """
        Get initial value for partial jacobian matrix
        
        :returns: partial jacobian
        """
        return np.zeros((2,4))


    def init_state_covariance(self) -> np.ndarray:
        """
        Get initial value for battery state covariance
        
        :returns: battery covariance
        """
        return np.diag([1, 1, 1, 1])

