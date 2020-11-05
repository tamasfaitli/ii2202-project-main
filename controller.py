class Controller:
    def __init__(self, dt=0.01):
        self.dt = dt

    def calculate_action(self, feedback, ref=0, ):
        raise NotImplementedError("Controller action calculation is not implemented!")

    def reset(self):
        raise NotImplementedError("Controller reset function is not implemented!")



## class: PIDController
class PIDController(Controller):
    def __init__(self, gain_p=0.0, gain_i=0.0, gain_d=0.0, dt=0.01):
        super().__init__(dt)
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.gain_d = gain_d

        self.int_error = 0
        self.prev_error = 0

    def calculate_action(self, feedback, ref=0):
        # local variable to update and return
        action = 0

        # check if
        assert(len(feedback) == len(ref))

        # calculating error terms
        error = feedback[0]-ref[0]
        d_error = feedback[1] - ref[1]

        # PID action calculation
        action = self.gain_p*error + self.gain_i*self.int_error + self.gain_d*d_error

        # integrate error
        self.int_error = self.int_error + error

        # returning value
        return action

    def reset(self):
        self.int_error = 0
        self.prev_error = 0


## class: MPCController
class MPCController(Controller):
    def __init__(self):
        super().__init__()

## class: RFPTController
class RFPTController(Controller):
    def __init__(self):
        super().__init__()