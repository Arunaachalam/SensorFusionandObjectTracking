# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.q = params.q
        self.dt = params.dt
        self.dim_state = params.dim_state

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        return np.matrix([[1, 0, 0, self.dt, 0, 0],
                          [0, 1, 0, 0, self.dt, 0],
                          [0, 0, 1, 0, 0, self.dt],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]
                          ])
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        eq1 = self.dt * self.q
        eq2 = 1 / 2 * (self.dt ** 2) * self.q
        eq3 = 1 / 3 * (self.dt ** 3) * self.q

        return np.matrix([[eq3, 0, 0, eq2, 0, 0],
                          [0, eq3, 0, 0, eq2, 0],
                          [0, 0, eq3, 0, 0, eq2],
                          [eq2, 0, 0, eq1, 0, 0],
                          [0, eq2, 0, 0, eq1, 0],
                          [0, 0, eq2, 0, 0, eq1]
                          ])
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        F = self.F()
        Q = self.Q()
        x = F * track.x
        P = F * track.P * F.transpose() + Q

        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        X = track.x
        P = track.P
        H = meas.sensor.get_H(X)

        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H)
        K = P * H.transpose() * np.linalg.inv(S)
        I = np.identity(self.dim_state)

        x = X + K * gamma
        P = (I -K * H) * P

        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        x = track.x
        z = meas.z
        hx = meas.sensor.get_hx(x)

        return z - hx
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        P = track.P
        R = meas.R

        return (H * P * H.transpose()) + R
        
        ############
        # END student code
        ############ 