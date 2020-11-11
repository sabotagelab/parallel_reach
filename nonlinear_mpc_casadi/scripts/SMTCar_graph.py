#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt

import numpy as np
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
        self.last_delta_datalist_car1=[]
        self.last_delta_datalist_car2=[]
        self. car1_delta_data =[]
        self.car2_delta_data=[]
        self.data_dT = rospy.get_param('data_dT',0.1)  # time interval at which to run z3 solver on the accumulated data

        self.CAR1_DATA_INIT = False
        self.CAR2_DATA_INIT = False
        self.FIRST_RUN = True

        #SMT solver related variables
        self.carOne_x = Function('carOne_x', IntSort(), RealSort())
        self.carOne_y = Function('carOne_y', IntSort(), RealSort())
        self.carTwo_x = Function('carTwo_x', IntSort(), RealSort())
        self.carTwo_y = Function('carTwo_y', IntSort(), RealSort())

        self.eps = 10
        self.delta = 0.25
        self.deltaSquared =0.0625
        self.radius = 100
        self.diameterSquared = pow(self.radius * 2, 2)

        self.start_time = 0
        self.time_eps = 0.04  #40ms

        # Publishers
        # e.g...self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, AckermannDriveStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber(car1_pose_topic, PoseStamped, self.car1_pose_callback, queue_size=1)
        rospy.Subscriber(car2_pose_topic, PoseStamped, self.car2_pose_callback, queue_size=1)

    def reset_datalist(self):
        self.pose_datalist_car1 = []
        self.pose_datalist_car2 = []
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
        if (time.time()-self.start_time) >= (self.data_dT-self.time_eps):
            self.last_delta_datalist_car1.append(copy.deepcopy(self.current_pose_car1))
        if not self.CAR1_DATA_INIT:
            self.CAR1_DATA_INIT = True

    def car2_pose_callback(self, msg):
        '''acquire estimated pose of car2 from particle filter'''
        current_yaw = self.quaternion_to_euler_yaw(msg.pose.orientation)
        self.current_pose_car2 = [msg.pose.position.x, msg.pose.position.y, current_yaw]
        self.pose_datalist_car2.append(copy.deepcopy(self.current_pose_car2))
        if (time.time()-self.start_time) >= (self.data_dT-self.time_eps):
            self.last_delta_datalist_car2.append(copy.deepcopy(self.current_pose_car2))
        if not self.CAR2_DATA_INIT:
            self.CAR2_DATA_INIT = True

    def Z3abs(self,x):
        return If(x >= 0, x, -x)

    def fetch_car_datalist(self):
        car1_data = copy.deepcopy(self.pose_datalist_car1)
        car2_data = copy.deepcopy(self.pose_datalist_car2)
        print(len(car1_data), len(car2_data))
        self.reset_datalist()
        return car1_data,car2_data

    def fetch_delta_datalist(self):
        self.car1_delta_data = copy.deepcopy(self.last_delta_datalist_car1)
        self.car2_delta_data = copy.deepcopy(self.last_delta_datalist_car2)
        self.last_delta_datalist_car1 = []
        self.last_delta_datalist_car2 = []

    def update_car_datalist(self,car1_data,car2_data):
        car1_data = self.car1_delta_data + car1_data
        car2_data = self.car2_delta_data + car2_data
        return car1_data,car2_data

    def solveSMT(self,car1_data,car2_data):
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
                car1_data,car2_data=self.fetch_car_datalist()
                self.solveSMT(car1_data,car2_data)
                print("SMT-solve-time=",time.time()-smt_start_time)
        rospy.spin()

    def vary_segment_length(self):
        start_seg = 0.1
        seg_interval = 0.2
        segment_list = [start_seg+seg_interval*i for i in range(11)]
        print(segment_list)
        segment_list_idx = 0
        self.eps = 1
        self.time_eps = self.eps * 0.04
        data_count_per_seg = 5
        data_counter = 0
        self.data_dT = segment_list[segment_list_idx]
        print("Segment length=",self.data_dT)
        total_time_list =[]
        seg_time_list=[]
        self.start_time = time.time()
        while not rospy.is_shutdown():
            if data_counter==data_count_per_seg:
                data_counter=0
                total_time_list.append(copy.deepcopy(seg_time_list))
                print(seg_time_list)
                seg_time_list =[]
                print("Segment length=", self.data_dT)
                segment_list_idx += 1
                if segment_list_idx == len(segment_list):
                    break
                self.data_dT = segment_list[segment_list_idx]
            current_time = time.time()
            if self.CAR1_DATA_INIT and self.CAR2_DATA_INIT:
                if (current_time - self.start_time) >= self.data_dT :
                    self.start_time = current_time
                    smt_start_time = time.time()
                    car1_data, car2_data = self.fetch_car_datalist()

                    if not self.FIRST_RUN:
                        car1_data, car2_data = self.update_car_datalist(car1_data, car2_data)
                    else:
                        self.FIRST_RUN = False
                    self.fetch_delta_datalist()

                    self.solveSMT(car1_data, car2_data)
                    solve_time = time.time()-smt_start_time
                    seg_time_list.append(solve_time)
                    data_counter += 1
                    print("SMT-solve-time=", solve_time)
            else:
                self.start_time=time.time()

        final_time_list = np.average(total_time_list,axis=1)
        # final_time_list = np.amax(total_time_list, axis=1)
        print(total_time_list)
        print(final_time_list)
        self.plot_segment_data(segment_list,final_time_list)
        rospy.spin()

    def vary_epsilon(self):
        self.data_dT = 2.0
        start_eps = 1
        eps_interval = 1
        eps_list = [start_eps + eps_interval * i for i in range(25)]
        print(eps_list)
        eps_list_idx = 0
        self.time_eps = self.eps * 0.04
        self.start_time = time.time()
        total_time_list =[]
        eps_time_list=[]
        while not rospy.is_shutdown():
            current_time = time.time()
            if self.CAR1_DATA_INIT and self.CAR2_DATA_INIT:
                if (current_time - self.start_time) >= self.data_dT :
                    print("Data collected")
                    self.start_time = current_time
                    car1_data, car2_data = self.fetch_car_datalist()
                    for i in range(len(eps_list)):
                        self.eps = eps_list[eps_list_idx]
                        smt_start_time = time.time()
                        self.solveSMT(car1_data, car2_data)
                        solve_time = time.time()-smt_start_time
                        eps_time_list.append(solve_time)
                        print("SMT-solve-time=", solve_time)
                    break
            else:
                self.start_time=time.time()

        print(eps_time_list)
        self.plot_epsilon_data(eps_list,eps_time_list)
        rospy.spin()

    def plot_epsilon_data(self,x_data,y_data):
        x_data = np.array(x_data)*0.04
        plt.plot(x_data, y_data, 'k', linewidth=1.5, marker='o')
        # plt.ylim(-0.2, 0.8)
        plt.ylabel('Runtime in s')
        plt.xlabel('Epsilon in s')
        plt.title("Runtime vs Epsilon(Segment length = 2.0)")
        plt.show()

    def plot_segment_data(self,x_data,y_data):
        plt.plot(x_data, y_data, 'r', linewidth=1.5, marker='o')
        # plt.ylim(-0.2, 0.8)
        plt.ylabel('Runtime in s')
        plt.xlabel('Segment length in s')
        plt.title("Runtime vs Segment length(Epsilon = 0.04)")
        plt.show()




if __name__ == '__main__':
    smt_node = SMTCar()
    # smt_node.run_SMT_node()
    # smt_node.vary_segment_length()
    smt_node.vary_epsilon()
