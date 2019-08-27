import rospy
from sensor_msgs.msg import _Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf.transformations as transform
import copy
import matplotlib.pyplot as plt
import time


class TyreModel:
    def __init__(self):
        rospy.init_node('tyre_model_node')

        #Variables and constants
        self.acceleration = None
        self.angular_vel = None
        self.current_pose = None
        self.last_pose = None
        self.current_time = time.time()
        self.last_time = time.time()

        self.steer_angle = 0.0
        self.v_x = 0
        self.v_y = 0
        self.beta = 0.0  #slip angle
        self.r = 0.0    #yaw rate
        self.alpha_f= 0.0
        self.alpha_r = 0.0
        self.alpha_f_list=[]
        self.front_force_list=[]
        self.alpha_r_list=[]
        self.rear_force_list=[]

        self.WHEELBASE = 0.325
        self.L_front= 0.18
        self.L_rear = 0.145
        self.FRONT_MASS = rospy.get_param('front_mass', 1.12)
        self.CALC_RATE = 20.0
        self.I_z = 0.0

        self.REAR_MASS = 1.12
        self.TOTAL_MASS = 2.24
        self.display_basic_parameters()


        #Initialization flags to check whether all the data sources are present or not
        self.pf_init_flag = False
        self.imu_accel_init_flag = False
        self.imu_gyro_init_flag = False
        self.data_inflow_flag = False
        self.plot_data_flag = False


        #Publishers and subscribers
        rospy.Subscriber('/camera/accel_sample',_Imu,self.imu_accel_callback)
        rospy.Subscriber('/camera/gyro_sample',_Imu,self.imu_gyro_callback)
        rospy.Subscriber('/pf/viz/inferred_pose',PoseStamped,self.pf_pose_callback)
        rospy.Timer(rospy.Duration(60),self.timer_callback)


    def quaternion_to_euler_yaw(self,orientation):
        _, _, yaw = transform.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        return yaw


    def imu_accel_callback(self,msg):
        '''callback for imu accelerometer data'''
        self.acceleration = [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
        if not self.imu_accel_init_flag:
            self.imu_accel_init_flag = True


    def imu_gyro_callback(self,msg):
        '''callback for imu gyro data'''
        self.angular_vel = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
        if not self.imu_gyro_init_flag:
            self.imu_gyro_init_flag = True


    def pf_pose_callback(self,msg):
        ''''callback for getting estimated pose from particle filter'''
        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y
        pose_yaw = self.quaternion_to_euler_yaw(msg.pose.orientation)
        self.current_pose = [pose_x, pose_y, pose_yaw]
        self.last_time = copy.deepcopy(self.current_time)
        self.current_time = time.time()
        if not self.pf_init_flag:
            self.last_pose = copy.deepcopy(self.current_pose)
            self.pf_init_flag = True
        self.data_inflow_flag= True

    def get_moment_of_inertia(self):
        I_z = self.L_front*self.L_rear*self.TOTAL_MASS
        return I_z

    def get_Lf_Lr(self):
        '''calculate Lf:distance between front wheels and CG
                     Lr:distance between real wheels and CG '''
        Lf = self.WHEELBASE * (1 - self.FRONT_MASS/self.TOTAL_MASS)
        Lr = self.WHEELBASE * (1 - self.REAR_MASS/self.TOTAL_MASS)
        return Lf,Lr

    def display_basic_parameters(self):
        self.L_front,self.L_rear = self.get_Lf_Lr()
        self.I_z = self.get_moment_of_inertia()
        print("Total mass=",self.TOTAL_MASS)
        print("Front mass=",self.FRONT_MASS)
        print("Rear mass=",self.REAR_MASS)
        print("Moment of Inertia Iz=",self.I_z)
        print("L_front=",self.L_front)
        print("L_rear=",self.L_rear)

    def get_slip_angle(self,delta_time):
        '''calculate slip angle based on actual motion vs desired motion'''
        actual_direction = np.arctan2(self.current_pose[1]-self.last_pose[1],self.current_pose[0]-self.last_pose[0])
        true_velocity = np.linalg.norm(np.array([self.current_pose[0],self.current_pose[1]])-np.array([self.last_pose[0],self.last_pose[1]]))/delta_time
        self.last_pose = copy.deepcopy(self.current_pose)
        slip_angle_beta = self.current_pose[2] - actual_direction
        return slip_angle_beta,true_velocity


    def get_force_on_tyres(self):
        '''calculate the force exerted on the front and rear tyres while turning'''
        Fy_f = self.FRONT_MASS* self.acceleration[1]/np.cos(self.steer_angle)
        Fy_r = self.REAR_MASS* self.acceleration[1]
        return Fy_f,Fy_r

    def get_slip_angles_tyres(self):
        alpha_f = np.arctan((self.v_y + self.r*self.L_front)/self.v_x) - self.steer_angle
        alpha_r = np.arctan((self.v_y + self.L_rear* self.r)/self.v_x)
        return alpha_f,alpha_r

    def run_parameters_estimation(self):
        delta_time = self.current_time - self.last_time
        self.beta , vel = self.get_slip_angle(delta_time)
        self.v_x = vel * np.cos(self.beta)
        self.v_y = vel * np.sin(self.beta)
        alpha_f, alpha_r = self.get_slip_angles_tyres()
        fy_f, fy_r = self.get_force_on_tyres()
        self.alpha_f_list.append(alpha_f)
        self.alpha_r_list.append(alpha_r)
        self.front_force_list.append(fy_f)
        self.rear_force_list.append(fy_r)
        self.data_inflow_flag = False



    def plot_data(self):
        '''plots force vs slip angle to estimate cornering stiffness'''
        plt.figure(1)
        plt.subplot(211)
        plt.plot(self.alpha_f_list, self.front_force_list)
        plt.subplot(212)
        plt.plot(self.alpha_r_list, self.rear_force_list)
        plt.show()

    def timer_callback(self,event):
        self.plot_data_flag = True


if __name__ == '__main__':
    tyre_model = TyreModel()
    while not rospy.is_shutdown():
        if tyre_model.pf_init_flag and tyre_model.imu_accel_init_flag and tyre_model.imu_gyro_init_flag and not tyre_model.plot_data_flag:
            tyre_model.run_parameters_estimation()
        elif tyre_model.plot_data_flag:
            tyre_model.plot_data()
            break
        rospy.sleep(tyre_model.CALC_RATE)

