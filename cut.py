import sys
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


def plot_viz():
    global force_,pose,euler,initial_pose
    sleep(5)
    plt_force=Plot(200,'FORCE',opt=opt_force)
    plt_pose=Plot(200,'POSE',opt=opt_pose)

    while True:
        plt_force.plot(force_)
        plt_pose.plot(pose*1000-initial_pose[:3])

if __name__ == '__main__':
    """
    moving:是否让机械臂运动
    euler:是否在旋转角度开启阻抗控制
    plot:是否将运动和力用visdom打印
    """
    plot = True

    args = sys.argv[1:]
    if len(args) == 1:
        plot = str2bool(args[0])
    dashboard, move = connect_robot()
    dashboard.EnableRobot()
    dashboard.ClearError()
    dashboard.SpeedFactor(80)
    dashboard.SetSafeSkin(0)
    
    force=Force()

    force_thread = threading.Thread(target=force.get_force)
    force_thread.daemon = True
    force_thread.start()

    if plot:
        record = threading.Thread(target=plot_viz)
        record.daemon = True
        record.start()


    initial_pose = [141.932007,-541.146973,391.485504,90.798767,0.044857,0.014894]
    print(initial_pose)
    move.MovL(initial_pose[0],initial_pose[1],initial_pose[2],initial_pose[3],initial_pose[4],initial_pose[5])
    move.Sync()

    ic = IC(initial_pose =[initial_pose[0]/1000,initial_pose[1]/1000,initial_pose[2]/1000,initial_pose[3],initial_pose[4],initial_pose[5]])
    # ic.change_para(m = [200,2,200,200,2,2],d = [1200,25,1000,1200,12,12],k = [0,128,0,0,5,5])
    ic.change_para(m = [200,2,200,8,2,2],d = [1200,25,1000,120,12,12],k = [0,128,0,0,5,5])
    limit_min=[(initial_pose[0]-100)/1000,(initial_pose[1]-150)/1000,(initial_pose[2]-100)/1000]
    limit_max=[(initial_pose[0]+200)/1000,(initial_pose[1]+100)/1000,(initial_pose[2]+200)/1000]
    ic.set_limit(limit_min,limit_max)
    ic.set_forward_force(np.array([0,0,2,0,0,0]))
    # ic.change_para(m = [200,0.2,200,200,2,2],d = [1200,20,1000,1200,12,12],k = [0,800,0,0,5,5])
    # ic.change_para(m = [200,0.2,200,200,2,2],d = [1200,8,1000,1200,12,12],k = [0,250,0,0,5,5])
    while True:
        start_time = time.time()
        force_ = [force.force[1]/10,-force.force[2]/3,-force.force[0]/10,force.force[4]*10,-force.force[5]*10,-force.force[3]*10]
        # force_ = [force.force[1]/10,-force.force[2]/3,-force.force[4],force.force[4]*10,-force.force[5]*10,-force.force[3]*10]  #这里将z轴的力设置为旋转轴的力，因为z轴受力没法传给六维力传感器。
        pose, euler = ic.compute_admittance_ff(force_,True)
        # print(pose[0]*1000,pose[1]*1000,pose[2]*1000,90.798767,0.044857,0.014894)
        move.ServoP(pose[0]*1000,pose[1]*1000,pose[2]*1000,euler[0],initial_pose[4],initial_pose[5])
        # move.ServoP(pose[0]*1000,pose[1]*1000,initial_pose[2],euler[0],0.044857,0.014894)
        while time.time() - start_time < 0.008:
            pass