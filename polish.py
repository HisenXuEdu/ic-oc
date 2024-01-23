import sys
from util.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
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
# from util.plot import Plot
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
    sleep(2)
    plt_force=Plot(200,'FORCE')
    plt_pose=Plot(200,'POSE')

    while True:
        plt_force.plot(force_)
        plt_pose.plot(pose*1000-initial_pose[:3])


def generate_move(ic,step):
    i = 1
    while True:
        if ic.desired_pose_position_[1]>-0.38:
            i = -1
        if ic.desired_pose_position_[1]<-0.6:
            i = 1
        ic.move_single([ic.desired_pose_position_[0],ic.desired_pose_position_[1]+200/100000*i,ic.desired_pose_position_[2]])
        sleep(step)


if __name__ == '__main__':
    """
    moving:是否让机械臂运动
    euler:是否在旋转角度开启阻抗控制
    plot:是否将运动和力用visdom打印
    """
    moving = True
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
    dashboard.SetSafeSkin(0)
    dashboard.SpeedFactor(60)


    force=Force()

    force_thread = threading.Thread(target=force.get_force)
    force_thread.daemon = True
    force_thread.start()

    initial_pose = [138.360397,-472.066620,407.361847,-179.488663,0.264109,179.605057]
    initial_joint = [90.0, 0.0, 100.0, -10.0, -90.0, 0.0]
    move.MovL(initial_pose[0],initial_pose[1],initial_pose[2],initial_pose[3],initial_pose[4],initial_pose[5])
    move.Sync()

    ic = IC(initial_pose=[initial_pose[0] / 1000, initial_pose[1] / 1000, initial_pose[2] / 1000, initial_pose[3], initial_pose[4], initial_pose[5]])

    if plot:
        record = threading.Thread(target=plot_viz)
        record.daemon = True
        record.start()

    if moving:
        tra = threading.Thread(target=generate_move,args=(ic,0.01))
        tra.daemon = True
        tra.start()

    ic.set_forward_force(np.array([0,0,5,0,0,0]))
    ic.change_para(m=[2, 2, 100, 2, 2, 2], d=[32, 25, 2000, 12, 12, 12], k=[128, 128, 0, 5, 5, 5])

    while True:
        start_time = time.time()
        # wrench_external_ = [force[1]/10,-force[2]/3,-force[0]/10,force[4]*10,-force[5]*10,-force[3]*10]
        force_ = [-force.force[1],-force.force[0],-force.force[2],force.force[4]*10,force.force[3]*10,-force.force[5]*10]
        pose, euler = ic.compute_admittance_ff(force_)
        # print(pose[0]*1000,pose[1]*1000,pose[2]*1000,initial_pose[3],initial_pose[4],initial_pose[5])
        move.ServoP(pose[0]*1000,pose[1]*1000,pose[2]*1000,initial_pose[3],initial_pose[4],initial_pose[5])
        while time.time() - start_time < 0.016:
            pass