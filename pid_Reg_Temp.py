class PID (object):
    def __init__(self, kp, Ti, Td, SP, TP):
        self.kp = kp
        self.Ti = Ti
        self.Td = Td
        self.setpoint = SP
        self.Tp = TP

        self.error = 0.0
        self.last_error = 0.0
        self.integral_error = 0.0
        self.derivative_error = 0.0
        self.output = 0.0

    #Metoda zwraca sygnał sterujący
    def compute(self, pos):
        self.error = self.setpoint - pos
        self.integral_error += self.error * self.Tp
        self.derivative_error = (self.error - self.last_error) / self.Tp
        self.last_error = self.error

        if self.Ti != 0:
            self.ki = 1/self.Ti
        else:
            self.ki = 0

        self.output = self.kp * (self.error + self.integral_error * self.ki + self.Td*self.derivative_error)

        return self.output, self.error