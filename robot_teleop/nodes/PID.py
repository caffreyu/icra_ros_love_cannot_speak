class PID:
    _kp = _ki = _kd = _output_max = _imax = last_error= 0

    def __init__(self, p=0, i=0, d=0, i_max=0, output_max=0):
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._i_max = abs(i_max)
        self._output_max = abs(output_max)
	self.last_error=0

    def calculate_pid(self, error):
        I_out = 0
        P_out = error * self._kp
        I_out += error * self._ki
        D_out = (error-self.last_error)*self._kd
        if I_out > self._i_max:
            I_out = self._i_max
        elif I_out < -self._i_max:
            Iout = -self._i_max
        output = P_out+I_out+D_out
        if output > self._output_max:
            output = self._output_max
        elif output < -self._output_max:
            output = -self._output_max
        self.last_error = error
        return output
