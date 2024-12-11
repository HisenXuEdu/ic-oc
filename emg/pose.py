import pytrigno as pytrigno
import pandas as pd
import numpy as np
from util.plot import *
import threading
from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt

class EMG:
    def __init__(self, channel=1, host='127.0.0.1'):
        self.channel = channel
        self.dev_emg = pytrigno.TrignoAccel(channel_range=(14,14), samples_per_read=18,
                    host=host)
        self.dev_emg.start()
        self.acc_data = np.array([[0.0, 0.0, 9.8], [0.0, 0.0, 9.8], [0.0, 0.0, 9.8]])  # 假设采样频率100Hz
        self.velocity = np.array([0.0 , 0.0, 0.0])
        self.position = np.array([0.0 , 0.0, 0.0])
        self.dt = 1/144

    
    def get_single(self):
        x = self.dev_emg.read()
        self.x = pd.DataFrame(x.T)
        return x
    
    def get_pose(self, x):
        global p
        

        # # 创建卡尔曼滤波器对象，状态空间为 6 维（3 维位置 + 3 维速度）
        # kf = KalmanFilter(dim_x=6, dim_z=3)

        # # 状态转移矩阵 (假设我们关心位置和速度)
        # kf.F = np.array([[1, 1, 0, 0, 0, 0],
        #                 [0, 1, 0, 0, 0, 0],
        #                 [0, 0, 1, 0, 0, 0],
        #                 [0, 0, 0, 1, 1, 0],
        #                 [0, 0, 0, 0, 1, 0],
        #                 [0, 0, 0, 0, 0, 1]])

        # # 测量矩阵
        # kf.H = np.array([[0, 0, 0, 0, 0, 0],
        #                 [0, 0, 0, 0, 0, 0],
        #                 [1, 0, 0, 0, 0, 0]])

        # # 过程噪声协方差矩阵
        # kf.Q = np.array([[1, 0, 0, 0, 0, 0],
        #                 [0, 1, 0, 0, 0, 0],
        #                 [0, 0, 1, 0, 0, 0],
        #                 [0, 0, 0, 1, 0, 0],
        #                 [0, 0, 0, 0, 1, 0],
        #                 [0, 0, 0, 0, 0, 1]])

        # # 测量噪声协方差矩阵
        # kf.R = np.array([[0.1, 0, 0],
        #                 [0, 0.1, 0],
        #                 [0, 0, 0.1]])

        # # 初始状态 (假设初始位置为 (0, 0, 0)，初始速度为 (0, 0, 0))
        # kf.x = np.array(x)

        # # 初始协方差矩阵
        # kf.P = np.eye(6)
        # for i in range(120):
        #     acc_data = self.get_single()
        #     print(acc_data)
        #     plt.plot(acc_data[0])
        #     plt.plot(acc_data[1])
        #     plt.plot(acc_data[2])
        #     plt.show()
        #     acc_data = acc_data.T
        #     print(acc_data)
        #     # 迭代卡尔曼滤波器
        #     for z in acc_data:
        #         kf.predict()       # 预测下一状态
        #         kf.update(z)       # 用测量值更新卡尔曼滤波器
        #         print(f"Updated state: {kf.x}")
        #     p = kf.x

        for i in range(120):
            acc_data = self.get_single()
            acc_data = acc_data.T
            print(acc_data)
            for i in range(1, len(acc_data)):
                # z轴加速度减去重力加速度
                acc_data[i][2] = acc_data[i][2] - 0.938
                self.velocity = self.velocity + acc_data[i] * 9.81 * self.dt
                self.position = self.position + self.velocity * self.dt
                print(self.position)
            p = self.position 

def plot_viz(pose):
    global p
    plt_pose=Plot(500,'POSE')
    while True:
        plt_pose.plot([p[0],p[1],p[2]])
        # plt_pose.plot([p[0],p[2],p[4]])


if __name__ == '__main__':
    print('start')
    emg = EMG(channel=1, host='127.0.0.1')
    pose_list = []
    p = [0, 0, 0, 0, 0, 0]
    x = [0, 0, 0, 0, 0, 0]

    record = threading.Thread(target=plot_viz, args=(pose_list,))
    record.daemon = True
    record.start()

    x = emg.get_pose(x)