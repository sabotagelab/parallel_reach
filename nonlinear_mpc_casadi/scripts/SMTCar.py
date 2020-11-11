#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

import copy
import time
from z3 import *

class SMTCar:
    def __init__(self):
        rospy.init_node("SMTcar")

        #topics to monitor
        car1_pose_topic= rospy.get_param('car1_pose_topic', '/car1_30m/pf/viz/inferred_pose')
        car2_pose_topic= rospy.get_param('car2_pose_topic', '/car2_5m/pf/viz/inferred_pose')

        self.current_pose_car1 = None
        self.current_pose_car2 = None
        self.pose_datalist_car1= []
        self.pose_datalist_car2= []
        self.data_dT = rospy.get_param('data_dT',0.1)  # time interval at which to run z3 solver on the accumulated data

        self.CAR1_DATA_INIT = False
        self.CAR2_DATA_INIT = False

        #SMT solver related variables
        self.carOne_x = Function('carOne_x', IntSort(), RealSort())
        self.carOne_y = Function('carOne_y', IntSort(), RealSort())
        self.carTwo_x = Function('carTwo_x', IntSort(), RealSort())
        self.carTwo_y = Function('carTwo_y', IntSort(), RealSort())

        self.eps = 10
        self.delta = 0
        self.deltaSquared = 0
        self.radius = 100
        self.diameterSquared = pow(self.radius * 2, 2)

        # Publishers
        # e.g...self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, AckermannDriveStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber(car1_pose_topic, PoseStamped, self.car1_pose_callback, queue_size=1)
        rospy.Subscriber(car2_pose_topic, PoseStamped, self.car2_pose_callback, queue_size=1)

    def reset_datalist(self):
        self.pose_datalist_car1 *= 0    #fastest way to clear the list. can also use list.clear() for python 3.3+ which is second fastest
        self.pose_datalist_car2 *= 0
        self.CAR1_DATA_INIT = False
        self.CAR2_DATA_INIT = False

    def quaternion_to_euler_yaw(self, orientation):
        _, _, yaw = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        return yaw

    def car1_pose_callback(self, msg):
        '''acquire estimated pose of car1 from particle filter'''
        current_yaw = self.quaternion_to_euler_yaw(msg.pose.orientation)
        self.current_pose_car1 = [msg.pose.position.x, msg.pose.position.y, current_yaw]
        self.pose_datalist_car1.append(copy.deepcopy(self.current_pose_car1))
        if not self.CAR1_DATA_INIT:
            self.CAR1_DATA_INIT = True

    def car2_pose_callback(self, msg):
        '''acquire estimated pose of car2 from particle filter'''
        current_yaw = self.quaternion_to_euler_yaw(msg.pose.orientation)
        self.current_pose_car2 = [msg.pose.position.x, msg.pose.position.y, current_yaw]
        self.pose_datalist_car2.append(copy.deepcopy(self.current_pose_car2))
        if not self.CAR2_DATA_INIT:
            self.CAR2_DATA_INIT = True

    def Z3abs(self,x):
        return If(x >= 0, x, -x)


    def solveSMT(self):
        car1_data = copy.deepcopy(self.pose_datalist_car1)
        car2_data = copy.deepcopy(self.pose_datalist_car2)
        print(len(car1_data),len(car2_data))
        self.reset_datalist()

        t1 = Int('t1')
        t2 = Int('t2')
        s = Solver()

        abstractTime_1 = 0
        abstractTime_2 = 0

        for i in range(len(car1_data)):
            s.add(self.carOne_x(abstractTime_1) == car1_data[i][0])
            s.add(self.carOne_y(abstractTime_1) == car1_data[i][1])
            abstractTime_1 += 1

        for i in range(len(car2_data)):
            s.add(self.carTwo_x(abstractTime_2) == car2_data[i][0])
            s.add(self.carTwo_y(abstractTime_2) == car2_data[i][1])
            abstractTime_2 += 1

        # Predicate for mutual separation; for all pair of events within the given epsilon, the distance between the two cars are greater than deltaSquared
        s.add(ForAll([t1, t2], (
            Implies(And(t1 >= 0, t1 < abstractTime_1, t2 >= 0, t2 < abstractTime_2, self.Z3abs(t1 - t2) <= self.eps),
                    self.deltaSquared < (((self.carTwo_x(t2) - self.carOne_x(t1)) * (
                            self.carTwo_x(t2) - self.carOne_x(t1))) + (
                                                 (self.carTwo_y(t2) - self.carOne_y(t1)) * (
                                                 self.carTwo_y(t2) - self.carOne_y(t1))))))))

        # Predicate for swarm radius;
        s.add(ForAll([t1, t2], (
            Implies(And(t1 >= 0, t1 < abstractTime_1, t2 >= 0, t2 < abstractTime_2, self.Z3abs(t1 - t2) <= self.eps),
                    self.diameterSquared >= (((self.carTwo_x(t2) - self.carOne_x(t1)) * (self.carTwo_x(t2) - self.carOne_x(t1))) + (
                                (self.carTwo_y(t2) - self.carOne_y(t1)) * (self.carTwo_y(t2) - self.carOne_y(t1))))))))

        print("Solution:....")
        print("abstractTime_1 :", abstractTime_1)
        print("abstractTime_2 :", abstractTime_2)
        result = s.check()
        print("Result=",result)
        if result == sat:
            # Modelling
            m = s.model()
            # print(m)

    def run_SMT_node(self):
        start_time = time.time()
        while not rospy.is_shutdown():
            current_time = time.time()
            if (current_time - start_time) >= self.data_dT and self.CAR1_DATA_INIT and self.CAR2_DATA_INIT:
                start_time = current_time
                smt_start_time= time.time()
                self.solveSMT()
                print("SMT-solve-time=",time.time()-smt_start_time)
        rospy.spin()



if __name__ == '__main__':
    smt_node = SMTCar()
    smt_node.run_SMT_node()
