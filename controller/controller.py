class Controller:
    def __init__(self, dt=0.01):
        self.dt = dt

    def calculate_action(self, feedback, ref=0, ):
        raise NotImplementedError("Controller action calculation is not implemented!")

    def reset(self):
        raise NotImplementedError("Controller reset function is not implemented!")
