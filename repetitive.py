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
        self.err_pub = rospy.Publisher('/error', Float64, queue_size=10)
        self.rate = rospy.Rate(20)
        self.volt_limit = 24

        # state matrice
        self.L = np.array([
            [ 0.0013],
            [ 0.00003461]
        ])

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
        
        
        # State Init
        self.u = 0
        self.y_est = 0
        self.x = np.array([
            [0],
            [0]
        ])


        # Integral Action
        self.rs_x = 0
        
        # Feedforward
        self.y1 = 0
        self.r1 = 0

        # RC
        self.err_l = [0]*54
        self.rc_l = [0]*54

        # Main Loop
        rospy.Subscriber('/Filtered_vel', Float64, self.get_filtered_vel)
        rospy.spin()

    def feedforward(self):
        # sin
        # self.yf1 = 100*math.sin(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti + 0.08))
        # self.yf2 = 100*math.sin(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti + 0.04))
        # self.y  = 100*math.sin(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti))

        # triangular
        self.yf1 = 100*signal.sawtooth(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti + 0.08), 0.5)
        self.yf2 = 100*signal.sawtooth(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti + 0.04), 0.5)
        self.y  = 100*signal.sawtooth(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti), 0.5)

        # step
        # self.yf1 = 200
        # self.yf2 = 200
        # self.y  = 200
        self.r  =   ( -0.1504*self.r1 + 
                    (self.yf1 - 0.8719*self.yf2 + 0.3714*self.y -0.0615*self.y1) )/0.2875
        self.r1 = self.r
        self.y1 = self.y


    def get_filtered_vel(self, vel):
        # self.feedforward()
        # self.r = 100*math.sin(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti))
        self.r = 100*signal.sawtooth(2*math.pi*self.f*(rospy.Time.now().to_sec() - self.ti), 0.5)
        self.y = self.r

        self.pub_input.publish(self.r)
        self.pub_ref.publish(self.y)
        self.obs_pub.publish(self.y_est)
        self.err_pub.publish(self.y - vel.data)

        self.repetitive(vel.data)
        self.integrator(vel.data)
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

    def repetitive(self, y_measured):
        self.err_l.pop(0)
        self.err_l.append(self.r - y_measured)

        self.r_rc = ( (4.915*self.rc_l[53] + 5.245*self.rc_l[52] + 2.368*self.rc_l[51] + 1.562*self.rc_l[50])
                    + (14.09*self.rc_l[4] - 4.915*self.rc_l[3] - 5.245*self.rc_l[2] - 2.368*self.rc_l[1] - 1.562*self.rc_l[0])
                    + (1*self.err_l[5] - 0.7438*self.err_l[4] + 0.4716*self.err_l[3] - 0.1987*self.err_l[2] + 0.07082*self.err_l[1] - 0.01303*self.err_l[0])
                    )/14.09

        

        self.rc_l.append(self.r_rc)
        self.rc_l.pop(0)
        # RC output is self.rc_l[53]

    def integrator(self, y_measured):
        self.rs_x = self.rs_x + (y_measured - self.r)
        self.rs_y = self.rs_x*-0.020411230150318 + self.rc_l[53]


    def state_estimator(self, y_measured):
        self.y_err = y_measured - self.y_est

        self.x = (self.u  * self.B) + (self.y_err * self.L) + (self.A.dot(self.x))
        self.y_est = self.C.dot(self.x)[0] 
        
    def state_feedback_controller(self):
        self.u = self.rs_y - self.K.dot(self.x)[0]
        

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