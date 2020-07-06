#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np
from scipy.signal import butter,filtfilt
import time

class vel_filtered():
    def __init__(self):
        # ---------------- Ros Initialization ----------------
        rospy.init_node('encoder_low_pass_filter', anonymous=True)
        self.pub = rospy.Publisher('/Filtered_vel', Float64, queue_size=10)
        self.rate = rospy.Rate(10)
        # ---------------- Ros Initialization ----------------

        # ---------------- Low Pass Filter ----------------
        self.Ts = 0.05
        self.fs = 1/self.Ts          
        self.cutoff = 1     
        self.nyq = 0.5 * self.fs     
        self.order = 2
        # ---------------- Low Pass Filter ----------------

        # ---------------- Parameters Initialization ----------------
        self.t0 = rospy.Time.now().to_sec()
        self.T_last = self.t0
        self.theta_list = []
        self.time = []
        self.omega_list = []
        self.o_list = []
        self.theta_last = 0
        # ---------------- Parameters Initialization ----------------

        # ---------------- Main Loop ----------------
        rospy.Subscriber('/Encoder', Float64, self.get_encoder)
        rospy.spin()
        # ---------------- Main Loop ----------------
        
    def get_encoder(self, msg):
        self.theta = msg.data * 0.75 * 0.5
        self.omega = (self.theta - self.theta_last)/0.04
        self.theta_last = self.theta
        self.lp_omega = (self.omega_list[-1] + 2*self.omega_list[-2] + self.omega_list[-3])/4
        self.pub.publish(self.lp_omega)
        

    def butter_lowpass_filter(self, data, cutoff, fs, order, nyq):
        self.normal_cutoff = cutoff / nyq
        # Get the filter coefficients 
        self.b, self.a = butter(order, self.normal_cutoff, btype='low', analog=False)
        self.y = filtfilt(self.b, self.a, data)
        return self.y

if __name__ == '__main__':
    try:
        a = vel_filtered()
        while not rospy.is_shutdown():
            pass

    except KeyboardInterrupt:
        pass

    finally:
        pass
        
