from controller.controller import Controller
import numpy as np

## class: RFPTController
class RFPTController(Controller):
    def __init__(self, params, model, dt):
        super().__init__("RFPT", dt)

        self.n_action = params["n_action"]
        self.l = params["l"]
        self.D = params["D"]
        self.A = params["A"]

        self.deformed_effect = np.zeros((self.n_action,1))
        self.int_error       = np.zeros((self.n_action,1))

        self.dyn_model  = model

        self.x_star     = 0.0
        self.__init_x_star()

    def calculate_action(self, feedback, reference):
        # local variable to update and return
        action = 0.0

        # update errors
        error = feedback[0] - reference[0]
        d_error = feedback[1] - reference[1]
        self.int_error += error * self.dt


        desired_effect  = self.__KinematicBlock(reference, self.int_error, error, d_error)
        self.deformed_effect = self.__AdaptiveBlock(feedback[2], desired_effect, self.deformed_effect)

        return self.dyn_model(feedback, self.deformed_effect)


    def __Fdef(self, x):
        return (x/2)+self.D

    def __init_x_star(self):
        for i in range(100):
            self.x_star = self.__Fdef(self.x_star)

    def __AdaptiveBlock(self, realized_effect, desired_effect, prev_deformed):
        h = realized_effect-desired_effect
        h_norm = np.linalg.norm(h)
        if h_norm > 1e-10:
            return (self.__Fdef(self.A*h_norm+self.x_star)-self.x_star)*(h/h_norm)+prev_deformed
        else:
            return prev_deformed

    def __KinematicBlock(self, reference, int_error, error, dot_error):
        return reference[2] + ((self.l**3)*int_error) + (3*(self.l**2)*error) + (3*self.l*dot_error)