# Since we are applying multiple PIDs (three of them, I think it is appropriate to make a class for it)
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.previous_error = 0.0
        
    def update(self, target_value, curr_value, init_dt):
        error = target_value - curr_value
        self.integral = self.integral + error * init_dt
        e_derivative = (error - self.previous_error) / init_dt
        
        self.previous_error = error
        
        updated_val = (self.kp * error) + (self.ki * self.integral) + (self.kp * e_derivative)
        return updated_val