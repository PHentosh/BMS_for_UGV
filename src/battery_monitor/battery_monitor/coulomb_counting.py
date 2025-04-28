

class CoulombCnt:
    last_SoC: float  # last calculated SoC
    Q:        float  # Battery capacity in Ah

    def __init__(self, Soc, Q):
        self.last_SoC = Soc
        self.Q = Q

    def step(self, i_in, dt):
        """
        Make one step of Coulomb counting calculations

        :param x:  state vector
        :param dt: time step in seconds

        :returns: SoC
        """
        self.last_SoC = self.last_SoC + i_in * (dt / 3600) / self.Q
        return self.last_SoC
