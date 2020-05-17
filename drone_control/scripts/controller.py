#!/usr/bin/env python

from threading import Lock

class PIController:
    def __init__(self, Kp, Ki, lower_limit=-1000.0, upper_limit=1000.0, windup_limit=1000.0):
        assert lower_limit < upper_limit
        assert windup_limit >= 0
        self.Kp = Kp
        self.Ki = Ki
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.windup_limit = windup_limit
        self.first_call = True
        self.integral = 0
        self.reference = 0
        self.last_call = 0
        self.last_err = 0
        self.param_mtx = Lock()
        self.reference_mtx = Lock()
    def set_Kp_Ki(self, Kp, Ki):
        with self.param_mtx:
            self.Kp = Kp
            self.Ki = Ki
        return
    def saturate(self, plant_input):
        result = plant_input
        if plant_input < self.lower_limit:
            result = self.lower_limit
        if plant_input > self.upper_limit:
            result = self.upper_limit
        return result
    def run(self, feedback, timestamp_s, reference=None):
        if reference is not None:
            with self.reference_mtx:
                self.reference = reference
        else:
            with self.reference_mtx:
                reference = self.reference
        with self.param_mtx:
            Kp = self.Kp
            Ki = self.Ki
        if not self.first_call:
            err = reference - feedback
            self.integral += (err + self.last_err) * (timestamp_s - self.last_call) / 2.0
            if abs(self.integral) > self.windup_limit:
                self.integral = self.windup_limit * self.integral / abs(self.integral)
        else:
            self.first_call = False
            err = reference - feedback
            self.integral = 0
        self.last_err = err
        self.last_call = timestamp_s
        return self.saturate(Kp * err + Ki * self.integral)
