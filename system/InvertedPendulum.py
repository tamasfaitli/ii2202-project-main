from system.system import System

class InvertedPendulum4DOF(System):
    def __init__(self, dt=0.01):
        super().__init__(4, dt)

