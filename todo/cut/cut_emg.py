import sys
import os

#5.1
# 将父级目录加入到import的path中
current_dir = os.path.dirname(__file__)
parent_dir = os.path.dirname(current_dir)
pparent_dir = os.path.dirname(parent_dir)
sys.path.append(pparent_dir)

from util.test import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ic.ic import IC
from ic.force import Force
import threading
from visdom import Visdom
from util.plot import Plot
from util.plot import *
from util.util import *
from emg.emg import *
from model.LSTM import *

def connect_robot():
    try:
        ip = "192.168.5.1"
        dashboardPort = 29999
        movePort = 30003
        # print("正在建立连接...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        # print(">.<move连接成功>!<")
        return dashboard, move
    except Exception as e:
        print(":(move连接失败:(")
        raise e


# def plot_viz():
#     global force_,pose,euler,initial_pose,K_list
#     sleep(5)
#     plt_force=Plot(200,'FORCE',opt=opt_force)
#     plt_pose=Plot(200,'POSE',opt=opt_pose)
#     start = time.time()

#     while True:
#         plt_force.plot(force_)
#         plt_pose.plot(pose*1000-initial_pose[:3])

def plot_viz():
    global force_,pose,euler_,initial_pose,K_list
    sleep(9)
    plt_force=Plot(200,'FORCE')
    plt_pose=Plot(200,'POSE')
    plt_k = Plot(200, 'K')
    force_list=[]
    pose_list=[]
    euler_list=[]

    while True:
        plt_force.plot(force_)
        plt_pose.plot(pose*1000-initial_pose[:3])
        plt_k.plot(1000 - (K_list[-1][0]*10000000-400)/2.5)
        
        force_list.append(force_)
        pose_list.append(pose*1000-initial_pose[:3]+euler_)
        euler_list.append([euler_[0],euler_[1],euler_[2]])
        print(len(force_list))
        if(len(force_list)>500):
            force_list = pd.DataFrame(force_list, columns=None)
            force_list.to_csv('./Data/FORCE6.csv', index=None)
            pose_list = pd.DataFrame(pose_list, columns=None)
            pose_list.to_csv('./Data/POSE6.csv', index=None)
            euler_list = pd.DataFrame(euler_list, columns=None)
            euler_list.to_csv('./Data/EULER6.csv', index=None)
            k_list = pd.DataFrame(K_list, columns=None)
            k_list.to_csv('./Data/K6.csv', index=None)
            break

def change_para():
    global emg, ic
    while True:
        data_EMG = emg.get_single()
        K_list.append(data_EMG)
        print(data_EMG)
        if data_EMG[1]<0.00005:
            print(1)
            # ic.change_para(m = [200,10,200,200,2,2],d = [1200,400,1000,1200,12,12],k = [0,900,0,0,5,5])
            # ic.change_para(m = [200,10,200,200,2,2],d = [250,400,1000,1200,12,12],k = [0,1500,0,0,5,5])
            # ic.change_para(m = [200,10,200,200,2,2],d = [250,250,1000,1200,12,12],k = [0,2000,0,0,5,5])
            ic.change_para(m = [200,10,200,200,2,2],d = [250,100,1000,1200,12,12],k = [0,500,0,0,5,5])
        else:
            print(2)
            ic.change_para(m = [200,2,200,8,2,2],d = [250,40,1000,120,12,12],k = [0,228,0,0,5,5])
            # ic.change_para(m = [200,10,200,200,2,2],d = [250,100,1000,1200,12,12],k = [0,500,0,0,5,5])


    while True:
        k = emg.get_single()
        print(k)
        m = [2,2,2,2,2,2]
        M_ = np.diag(m)
        k = [100,100,100,5,5,5]
        K_ = np.diag(k)
        d = [32,32,32,2,2,2]
        
        

        channel=1
        dev_emg = pytrigno.TrignoEMG(channel_range=(0,channel-1), samples_per_read=500,
                            host='127.0.0.1')
        dev_emg.start()
        data_EMG = dev_emg.read()
        while True:
            data_EMG = dev_emg.read()
            data_EMG = np.abs(data_EMG)
            data_EMG = np.mean(data_EMG)
            print(data_EMG)
            m = [200,2,200,8,2,2]
            M_ = np.diag(m)
            d = [1200,25,1000,120,12,12]
            D_ = np.diag(d)
            k = [0,128,0,0,5,5]
            K_ = np.diag(k)
    

def change_para1():
    global emg, ic
    while True:
        data_EMG = emg.get_single()
        K_list.append(data_EMG)
        print(data_EMG)
        kc = (1000 - (K_list[-1][0]*10000000-400))/2
        dc = kc/5
        if(kc<200): 
            kc=100
        if(dc<40):
            dc=20
        print(kc, dc)
        ic.change_para(m = [200,10,200,200,2,2],d = [250,dc,1000,1200,12,12],k = [0,kc,0,0,5,5])


def change_para_emg():
        global emg, ic
        while True:
            data_EMG = emg.get_single_network()
            # K_list.append(data_EMG)
            print(data_EMG)
            res = result(model, x, y)
            kc = (1000 - (K_list[-1][0]*10000000-400))/2
            dc = kc/5
            if(kc<200): 
                kc=100
            if(dc<40):
                dc=20
            print(kc, dc)
            ic.change_para(m = [200,10,200,200,2,2],d = [250,dc,1000,1200,12,12],k = [0,kc,0,0,5,5])


if __name__ == '__main__':
    """
    moving:是否让机械臂运动
    euler:是否在旋转角度开启阻抗控制
    plot:是否将运动和力用visdom打印
    """
    plot = True

    K_list = []


    win_len = 20
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = MyModel(6, win_len,1)
    model.to(device)
    checkpoint = torch.load('model_path/LSTMi1o1.pth')
    model.load_state_dict(checkpoint['net'])



    args = sys.argv[1:]
    if len(args) == 1:
        plot = str2bool(args[0])
    dashboard, move = connect_robot()
    dashboard.EnableRobot()
    dashboard.ClearError()
    dashboard.SpeedFactor(60)
    dashboard.SetSafeSkin(0)
    
    force=Force()

    force_thread = threading.Thread(target=force.get_force)
    force_thread.daemon = True
    force_thread.start()

    if plot:
        record = threading.Thread(target=plot_viz)
        record.daemon = True
        record.start()


    

    initial_pose = [141.932007,-441.146973,110.485504,90.798767,0.044857,0.014894]
    print(initial_pose)
    move.MovL(initial_pose[0],initial_pose[1],initial_pose[2],initial_pose[3],initial_pose[4],initial_pose[5])
    move.Sync()

    ic = IC(initial_pose =[initial_pose[0]/1000,initial_pose[1]/1000,initial_pose[2]/1000,initial_pose[3],initial_pose[4],initial_pose[5]])
    # ic.change_para(m = [200,2,200,200,2,2],d = [1200,25,1000,1200,12,12],k = [0,128,0,0,5,5])
    ic.change_para(m = [200,2,200,8,2,2],d = [1200,25,1000,120,12,12],k = [0,128,0,0,5,5])
    limit_min=[(initial_pose[0]-100)/1000,(initial_pose[1]-100)/1000,(initial_pose[2]-100)/1000]
    limit_max=[(initial_pose[0]+200)/1000,(initial_pose[1]+100)/1000,(initial_pose[2]+200)/1000]
    ic.set_limit(limit_min,limit_max)
    ic.set_forward_force(np.array([0,0,2,0,0,0]))

    emg = Emg_S(channel=3, host='127.0.0.1')
    para_thread = threading.Thread(target=change_para)
    para_thread.daemon = True
    para_thread.start()

    while True:
        start_time = time.time()
        force_ = [force.force[1]/10,-force.force[2]/3,-force.force[0]/3,force.force[4]/5,-force.force[5]/5,-force.force[3]/5]
        # force_ = [force.force[1]/10,-force.force[2]/3,-force.force[4],force.force[4]*10,-force.force[5]*10,-force.force[3]*10]  #这里将z轴的力设置为旋转轴的力，因为z轴受力没法传给六维力传感器。
        pose, euler_ = ic.compute_admittance_ff(force_,True)
        # print(pose[0]*1000,pose[1]*1000,pose[2]*1000,90.798767,0.044857,0.014894)
        move.ServoP(pose[0]*1000,pose[1]*1000,pose[2]*1000,initial_pose[3],initial_pose[4],initial_pose[5])
        # move.ServoP(pose[0]*1000,pose[1]*1000,initial_pose[2],euler[0],0.044857,0.014894)
        while time.time() - start_time < 0.008:
            pass