from controller.controller import Controller

## class: PIDController
class PIDController(Controller):
    def __init__(self, params, dt=0.01):
        super().__init__(dt)
        self.gain_p = params['proportional']
        self.gain_i = params['integral']
        self.gain_d = params['derivative']

        self.int_error  = 0
        self.prev_error = 0

    def calculate_action(self, feedback, ref=0):
        # check if
        assert(len(feedback) == len(ref))

        # local variable to update and return
        action = 0.0

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