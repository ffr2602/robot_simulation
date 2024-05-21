
class Pid:
    i_err = 0
    d_err = 0
    last_err = 0
    windup = 0.2
    limit = 0.6

    def __init__(self, kp=0, ki=0, kd=0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def pid(self, error) -> float:
        if self.i_err > self.windup:
            self.i_err = 0
        self.d_err = error - self.last_err
        self.last_err = error
        self.i_err = self.i_err + error
        result_pid = (self.kp * error) + (self.ki * self.i_err) + (self.kd * self.d_err)
        if result_pid >= 0:
            return self.limit if result_pid > self.limit else result_pid
        else:
            return -self.limit if result_pid < -self.limit else result_pid

    def reset_err(self):
        self.i_err = 0
        self.d_err = 0
        self.last_err = 0
