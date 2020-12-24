class Controller:
    def __init__(self, name, dt=0.01):
        self.name = name
        self.dt = dt

    def calculate_action(self, feedback, reference):
        raise NotImplementedError("Controller action calculation is not implemented!")

    def reset(self):
        raise NotImplementedError("Controller reset function is not implemented!")

    def get_name(self):
        return self.name
