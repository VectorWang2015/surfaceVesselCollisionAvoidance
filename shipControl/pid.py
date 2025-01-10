# encoding: utf-8
# author: vectorwang@hotmail.com
# license: MIT

r"""
       _     _ 
 _ __ (_) __| |
| '_ \| |/ _` |
| |_) | | (_| |
| .__/|_|\__,_|
|_|
"""
from collections import deque


class PID:
    def __init__(
            self,
            kp: float,
            ki: float,
            kd: float,
            buffer_size: int=20,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.buffer_size = buffer_size
        self.error_buffer = deque(maxlen=self.buffer_size)
    
    def store_error(self, error: float):
        self.error_buffer.append(error)
        if len(self.error_buffer) > self.buffer_size:
            del(self.error_buffer[0])

    def calc_p(self) -> float:
        return self.kp * self.error_buffer[-1]

    def calc_i(self) -> float:
        return self.ki * sum(self.error_buffer) / self.buffer_size

    def calc_d(self) -> float:
        if len(self.error_buffer) < 2:
            return 0
        return self.kd * (self.error_buffer[-1] - self.error_buffer[-2])

    def control(self, desire_value, current_value) -> float:
        current_error = desire_value - current_value
        self.store_error(current_error)

        return self.calc_p() + self.calc_d() + self.calc_i()