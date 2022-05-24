
class PID(object):

    def __init__(self, kp = 0., ki = 0., kd = 0.):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.total_err = 0.
        self.last_time = 0.
        self.last_err  = 0.
        
    def change_gains(self, kp = 0., ki = 0., kd = 0.):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def calc_control(self, error, time):
        #calculate p
        sum_p = self.kp * error
        
        #calculate i
        if self.last_time == 0.:
            self.last_time = time
        dt = time - self.last_time
        self.total_err = self.total_err + (error * dt)
        sum_i = self.ki * self.total_err
        
        #calculate d
        de = error - self.last_err
        if dt > 0:
            dedt = de / dt
        else:
            dedt = 0.
        sum_d = self.kd * dedt
        
        all_sum = sum_p + sum_i + sum_d
        
        self.last_time = time
        self.last_err = error
        
        return all_sum
        

