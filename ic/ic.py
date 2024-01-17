import numpy as np
import socket
import struct
import time

class IC():
    def __init__(self, m=[2, 2, 2, 2, 2, 2], d=[32, 25, 12, 12, 12, 12], k=[128, 128, 100, 5, 5, 5],
                    initial_pose = [138.360397/1000,-472.066620/1000,407.361847/1000, 179.488663,0.264109,179.605057], forward_force=np.zeros(6)):

        # admittance parameters
        self.M_ = np.diag(m)
        self.D_ = np.diag(d)
        self.K_ = np.diag(k)
        self.arm_max_acc_ = 1

        self.desired_pose_position_ = np.array(initial_pose[0:3]) 
        self.desired_pose_euler_ = np.array(initial_pose[3:6]) 

        #sim
        self.arm_desired_twist_ = np.zeros(6)
        self.arm_desired_pose_ = np.array(initial_pose)
        

        self.forward_force=forward_force


        #limit
        self.limit_min=[-1000,-1000,-1000,-1000,-1000,-1000]
        self.limit_max=[1000,1000,1000,1000,1000,1000]

        # 计算循环的时间间隔
        # loop_rate = 20  # Hz
        loop_rate = 75 # Hz
        self.sec = 1.0 / loop_rate

    

    def compute_admittance(self, force=np.zeros(6)) -> tuple[np.ndarray, np.ndarray]:
        """单步计算导纳API

        Args:
            force (_type_, optional): 当前接触力. Defaults to np.zeros(6).

        Returns:
            tuple[np.ndarray, np.ndarray]: pose和euler
        """
        error = np.zeros(6)
        error[0:3] = self.arm_desired_pose_[0:3]  - self.desired_pose_position_
        error[3:6] = self.arm_desired_pose_[3:6] - self.desired_pose_euler_
        coupling_wrench_arm = np.dot(self.D_, self.arm_desired_twist_) + np.dot(self.K_, error)
        arm_desired_accelaration = np.linalg.inv(self.M_) @ (-coupling_wrench_arm + force)
        a_acc_norm = np.linalg.norm(arm_desired_accelaration[0:3])
        if a_acc_norm > self.arm_max_acc_:
            # print("Admittance generates high arm acceleration! norm:", a_acc_norm)
            arm_desired_accelaration[0:3] *= (self.arm_max_acc_ / a_acc_norm)
        self.arm_desired_twist_ += arm_desired_accelaration * self.sec  #进行速度迭代并记录
        self.arm_desired_pose_ += self.arm_desired_twist_ * self.sec  #这里应该用arm_desired_twist_+当前速度
        pose = self.arm_desired_pose_[0:3] 
        euler = self.arm_desired_pose_[3:6] 
        return pose, euler
    
    def compute_admittance_l(self, force=np.zeros(6)) -> tuple[np.ndarray, np.ndarray]:
        """带限位单步计算导纳API

        Args:
            limit_min (np.ndarray): 限位最小值
            limit_max (np.ndarray): 限位最大值
            force (_type_, optional): 当前接触力. Defaults to np.zeros(6).

        Returns:
            tuple[np.ndarray, np.ndarray]: pose和euler
        """
        error = np.zeros(6)
        error[0:3] = self.arm_desired_pose_[0:3]  - self.desired_pose_position_
        error[3:6] = self.arm_desired_pose_[3:6] - self.desired_pose_euler_
        coupling_wrench_arm = np.dot(self.D_, self.arm_desired_twist_) + np.dot(self.K_, error)
        arm_desired_accelaration = np.linalg.inv(self.M_) @ (-coupling_wrench_arm + force)
        a_acc_norm = np.linalg.norm(arm_desired_accelaration[0:3])
        if a_acc_norm > self.arm_max_acc_:
            # print("Admittance generates high arm acceleration! norm:", a_acc_norm)
            arm_desired_accelaration[0:3] *= (self.arm_max_acc_ / a_acc_norm)
        self.arm_desired_twist_ += arm_desired_accelaration * self.sec  #进行速度迭代并记录
        self.arm_desired_pose_ += self.arm_desired_twist_ * self.sec  #这里应该用arm_desired_twist_+当前速度
        pose = self.arm_desired_pose_[0:3] 
        pose = self.__limit(pose) #这里仅实现了对xyz的限位
        euler = self.arm_desired_pose_[3:6] 
        return pose, euler
    
    def set_limit(self, limit_min:np.ndarray, limit_max:np.ndarray):
        """设置限位

        Args:
            limit_min (np.ndarray): 限位最小值
            limit_max (np.ndarray): 限位最大值
        """
        self.limit_min=limit_min
        self.limit_max=limit_max

    def __limit(self,val):  #__代表私有成员
        for i in range(len(val)):
            if val[i] < self.limit_min[i]:
                val[i] = self.limit_min[i]
                self.arm_desired_twist_[i] = 0
            if val[i] > self.limit_max[i]:
                val[i] = self.limit_max[i]
                self.arm_desired_twist_[i] = 0
        return val

    def change_para(self, m, d, k):
        self.M_ = np.diag(m)
        self.D_ = np.diag(d)
        self.K_ = np.diag(k)

    def forward_force(self, force=np.zeros(6)):
        # m = [2,2,2,0.5,0.5,0.5]
        # M_ = np.diag(m)
        # k = [128,128,0,0.1,0.1,0.1]
        # K_ = np.diag(k)
        # d = [32,25,250,2,2,2]
        # D_ = np.diag(d)
        error = np.zeros(6)
        error[0:3] = self.arm_desired_pose_[0:3]  - self.desired_pose_position_
        error[3:6] = self.arm_desired_pose_[3:6] - self.desired_pose_euler_
        coupling_wrench_arm = np.dot(self.D_, self.arm_desired_twist_) + np.dot(self.K_, error)
        force -= self.forward_force
        arm_desired_accelaration = np.linalg.inv(M_) @ (-coupling_wrench_arm + force)
        a_acc_norm = np.linalg.norm(arm_desired_accelaration[0:3])
        if a_acc_norm > self.arm_max_acc_:
            # print("Admittance generates high arm acceleration! norm:", a_acc_norm)
            arm_desired_accelaration[0:3] *= (self.arm_max_acc_ / a_acc_norm)
        self.arm_desired_twist_ += arm_desired_accelaration * self.sec  #进行速度迭代并记录
        self.arm_desired_pose_ += self.arm_desired_twist_ * self.sec  #这里应该用arm_desired_twist_+当前速度
        pose = self.arm_desired_pose_[0:3] 
        euler = self.arm_desired_pose_[3:6] 
        return pose, euler

