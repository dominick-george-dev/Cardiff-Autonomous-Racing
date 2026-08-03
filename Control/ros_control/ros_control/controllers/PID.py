class PID:
    def __init__(self, K_P:float, K_I:float, K_D:float, set_point:float, loop_interval_time:float) -> None:
        self.set_params(K_P=K_P, K_I=K_I, K_D=K_D)

        self.set_point = set_point

        self.dt = loop_interval_time

    def control_function(self, measured_value:float) -> float:
        """
        Actual PID implementation
        """
        return 0.0



    def set_params(self, K_P:float, K_I:float, K_D:float) -> None:
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D

    def get_params(self) -> tuple[float, float, float]:
        return (self.K_P, self.K_I, self.K_D)


    def set_set_point(self, set_point:float) -> None:
        self.set_point = set_point

    def get_set_point(self) -> float:
        return self.set_point