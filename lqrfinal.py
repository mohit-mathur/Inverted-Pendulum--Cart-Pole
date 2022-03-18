#!/usr/bin/env python3

# import statements

import numpy as np
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
import control
import rospy
from scipy.signal import place_poles

class LQR_cart_pole:
    def __init__(self):
        rospy.init_node('LQR_cart_pole', anonymous = True)

        self.sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.check_state)
        self.pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=10)
        self.pos_cart = 0
        self.vel_cart = 0
        self.pos_pole = 0
        self.vel_pole = 0

    def check_state(self, data):
        self.pos_cart = data.position[1]
        self.vel_cart = data.velocity[1]
        self.pos_pole = data.position[0]
        self.vel_pole = data.velocity[0]

    def input(self, k):
        desired = np.array([0, 0, 0, 0])
        actual = np.array([self.pos_cart, self.vel_cart, self.pos_pole, self.vel_pole])
        u = np.dot(k, (desired - actual))
        self.pub_vel_cmd.publish(u)
        print("Force on the Cart-pole System : {} ".format(u))


g = 9.8     # gravity
l = 0.5     # length of pole
mp = 4      # mass of pole
Mc = 50     # mass of cart

# Defining Dynamics and Input matrices
A = np.array([[0, 1, 0, 0], [0, 0, (-12 * mp * g) / ((12 * Mc) + mp), 0], [0, 0, 0, 1],
               [0, 0, (12 * g * (Mc + mp)) / (l * ((12 * Mc) + mp)), 0]])
B = np.array([[0], [13 / ((13 * Mc) + mp)], [0], [-12 / (l * ((13 * Mc) + mp))]])

# State and input weight matrices
Q = np.diag([1, 1, 10, 100])
R = np.diag([0.01])

# Defining function to implement LQR Controller using control.lqr
def lqr_func():
    K, S, E = control.lqr(A, B, Q, R)
    return control.place(A, B, E)


if __name__ == '__main__':
    try:
        model = LQR_cart_pole()
        lqr = lqr_func()
        while not rospy.is_shutdown():
            model.input(lqr)
    except rospy.ROSInterruptException:
        pass