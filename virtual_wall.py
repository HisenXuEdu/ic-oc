import sys
from util.test import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ic.ic import IC
import ic.force as force
import threading
from visdom import Visdom
from util.plot import Plot
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
    sleep(12)
    plt_force=Plot(200,'FORCE')
    plt_pose=Plot(200,'POSE')

    while True:
        plt_force.plot(force_)
        plt_pose.plot(pose*1000-initial_pose[:3])

if __name__ == '__main__':
    """
    moving:是否让机械臂运动
    euler:是否在旋转角度开启阻抗控制
    plot:是否将运动和力用visdom打印
    """
    moving = False
    euler = False
    plot = True

    args = sys.argv[1:]
    if len(args) == 1:
        moving = str2bool(args[0])
    elif len(args) == 2:
        moving = str2bool(args[0])
        euler = str2bool(args[1])
    elif len(args) == 3:
        moving = str2bool(args[0])
        euler = str2bool(args[1])
        plot = str2bool(args[2])


    dashboard, move = connect_robot()
    dashboard.EnableRobot()
    dashboard.ClearError()
    dashboard.SpeedFactor(60)
    dashboard.SetSafeSkin(0)
    s = force.connect_force()
    

    force_thread = threading.Thread(target=force.get_force, args=(s,))
    force_thread.daemon = True
    force_thread.start()

    if plot:
        record = threading.Thread(target=plot_viz)
        record.daemon = True
        record.start()

    initial_pose = [138.360397,-472.066620,407.361847,-179.488663,0.264109,179.605057]
    # initial_pose = [-472.360397,-135.066620,407.361847,-179.488663,0.264109,89.605057]
    initial_joint = [90.0, 0.0, 100.0, -10.0, -90.0, 0.0]
    print(initial_pose)
    move.MovL(initial_pose[0],initial_pose[1],initial_pose[2],initial_pose[3],initial_pose[4],initial_pose[5])
    sleep(10)

    ic = IC(initial_pose=[initial_pose[0] / 1000, initial_pose[1] / 1000, initial_pose[2] / 1000, initial_pose[3], initial_pose[4], initial_pose[5]])
    limit_min=[(initial_pose[0]-100)/1000,(initial_pose[1]-100)/1000,(initial_pose[2]-100)/1000]
    limit_max=[(initial_pose[0]+200)/1000,(initial_pose[1]+200)/1000,(initial_pose[2]+200)/1000]
    ic.set_limit(limit_min,limit_max)

    if euler:
        while True:
            start_time = time.time()
            # wrench_external_ = [force[1]/10,-force[2]/3,-force[0]/10,force[4]*10,-force[5]*10,-force[3]*10]
            force_ = [-force.force[1]/10,-force.force[0]/10,-force.force[2]/10,force.force[4]*10,force.force[3]*10,-force.force[5]*10]  #这里将z轴的力设置为旋转轴的力，因为z轴受力没法传给六维力传感器。
            pose, euler = ic.compute_admittance(force_)
            # print(initial_pose[0],initial_pose[1],initial_pose[2],euler[0],euler[1],euler[2])
            move.ServoP(initial_pose[0],initial_pose[1],initial_pose[2],euler[0],euler[1],euler[2])
            while time.time() - start_time < 0.01:
                pass
    else:
        print("start")
        while True:
            start_time = time.time()
            # wrench_external_ = [force[1]/10,-force[2]/3,-force[0]/10,force[4]*10,-force[5]*10,-force[3]*10]
            force_ = [-force.force[1],-force.force[0],-force.force[2],force.force[4]*10,force.force[3]*10,-force.force[5]*10]  #这里将z轴的力设置为旋转轴的力，因为z轴受力没法传给六维力传感器。
            pose, euler = ic.compute_admittance_l(force_)
            # print(pose[0]*1000,pose[1]*1000,pose[2]*1000,initial_pose[3],initial_pose[4],initial_pose[5])
            move.ServoP(pose[0]*1000,pose[1]*1000,pose[2]*1000,initial_pose[3],initial_pose[4],initial_pose[5])
            while time.time() - start_time < 0.016:
                pass