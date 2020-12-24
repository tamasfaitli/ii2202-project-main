from controller.controller import Controller

## class: PIDController
class PIDController(Controller):
    def __init__(self, params, dt=0.01):
        super().__init__("PID", dt)
        self.gain_p = params['proportional']
        self.gain_i = params['integral']
        self.gain_d = params['derivative']

        self.int_error  = 0
        self.prev_error = 0

    def calculate_action(self, feedback, reference):
        # local variable to update and return
        action  = 0.0

        # calculating error terms
        error   = feedback[0]-reference[0]
        d_error = feedback[1] - reference[1]
        self.int_error += self.dt*error

        # PID action calculation
        action  = self.gain_p*error + self.gain_i*self.int_error + self.gain_d*d_error

        # returning value
        return action

    def reset(self):
        self.int_error = 0
        self.prev_error = 0