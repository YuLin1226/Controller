#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np
import math 
from scipy import signal

class state_observer():
    def __init__(self):
        
        # ROS Setup
        rospy.init_node('state_observer_node', anonymous=True)
        self.pub = rospy.Publisher('/PWM', Float64, queue_size=10)
        self.pub_input = rospy.Publisher('/Input', Float64, queue_size=10)
        self.pub_ref = rospy.Publisher('/ref', Float64, queue_size=10)
        self.obs_pub = rospy.Publisher('/Observer', Float64, queue_size=10)
        self.rate = rospy.Rate(20)
        self.volt_limit = 24

        # state matrice
        self.L = np.array([
            [ 0.0013],
            [ 0.00003461]
        ])

        # self.K = np.array([
        #     [8.189843154009619,-87.905426559284310]
        # ])

        self.K = np.array([
            [[15.122486614033923,5.549227174054342e+02]]
        ])

        self.A = np.array([
            [0.6605  ,   -11.6],
            [0.01444 ,  -0.03003]
        ])

        self.B = np.array([
            [0.03465],
            [0.0004226]
        ])
        
        self.C = np.array([
            [0 , 33330]
        ])

        # Tracking Input [deg / s]
        self.ti = rospy.Time.now().to_sec()
        self.f  = 0.5
        self.reference = 200
        self.r = self.reference
        
        self.r1 = 0
        self.r2 = 0
        # State Init
        self.u = 5.5
        self.y_est = 0
        self.y_eps_last = 0
        self.x_last = np.array([
            [0],
            [0]
        ])


        # Integral Action
        self.x_intergral_last = 0

        # Feedforward
        self.yf = self.reference
        self.y  = self.reference
        self.y1 = 0
        self.y2 = 0
        # for i in range(100):
        #     self.feedforward()
        #     print("Feedforward Output r: ", self.r)



        # Main Loop
        rospy.Subscriber('/Filtered_vel', Float64, self.get_filtered_vel)
        rospy.spin()
# 

    def feedforward(self):
        # sin
        # self.yf = 100*math.sin(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti + 0.04))
        # self.y  = 100*math.sin(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti))

        # triangular
        # self.yf = 100*signal.sawtooth(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti + 0.04), 0.5)
        # self.y  = 100*signal.sawtooth(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti), 0.5)

        # step
        self.yf = 200
        self.y  = 200
        self.r  =   ( (6.429*self.r1 + 7.219*self.r2) + 
                    (self.yf - 0.8719*self.y + 0.3714*self.y1 -0.0615*self.y2) )/14.09
        self.r1 = self.r
        self.r2 = self.r1
        self.y1 = self.y
        self.y2 = self.y1


    def get_filtered_vel(self, vel):
        # self.feedforward()
        # self.r = 100*math.sin(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti))
        self.pub_input.publish(self.reference)
        self.pub_ref.publish(self.y)
        self.obs_pub.publish(self.y_est)
        self.state_estimator(vel.data)
        self.state_feedback_controller()
        self.saturation()
        self.gain = self.u *100/self.volt_limit
        self.pub.publish(self.gain)

        print('============================')
        print('Measurement y', vel.data)
        print('Observer y', self.y_est[0])
        # print('output error', self.y_err[0])
        print('Voltage', self.u)
        print('State', self.x)
        print('============================')

    def state_estimator(self, y_measured):
        self.y_err = y_measured - self.y_est

        # self.y_eps = self.y_eps_last + self.y_err
        self.x = (self.u  * self.B) + (self.y_err * self.L) + (self.A.dot(self.x_last))
        # self.x = (self.u  * self.B) + (self.y_eps_last * self.L) + (self.A.dot(self.x_last))
        self.y_est = self.C.dot(self.x)[0] 

        # self.y_eps_last = self.y_eps
        self.x_last = self.x
        
        # Integral Action
        # self.err_integral = self.x_intergral_last - (y_measured - self.r)*0.01
        self.err_integral = self.x_intergral_last + (y_measured - self.r)*1
        self.x_intergral_last = self.err_integral

    def state_feedback_controller(self):
        self.err = self.r - self.K.dot(self.x)[0]
        self.u = self.err + self.err_integral*-0.020411230150318
        

    def saturation(self):
        if self.u >= self.volt_limit:
            self.u = self.volt_limit
        elif self.u <= -self.volt_limit:
            self.u = -self.volt_limit
        


if __name__ == '__main__':
    try:
        a = state_observer()
        while not rospy.is_shutdown():
            pass

    except KeyboardInterrupt:
        pass

    finally:
        pass