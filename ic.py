from util.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from ic.ic_class import IC
import ic.force as force
import threading
from visdom import Visdom
import util.util

def ConnectRobot():
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


def pose_open():
    #传递的第一个参数是是否写入文件，第二个参数是是否用visdom可视化，默认均不开启
    os.system('python ./util/pose_record.py false true')


def fsm(dmp,last_pose,goal=np.array([])):
    pass
    intial = last_pose
    intial = np.array(intial)
    dmp.y0 = intial
    if(len(goal)!=0):
        goal = np.array(goal)
        dmp.goal = goal

def change_goal():
    #需要实现根据视觉变goal
    pass


if __name__ == '__main__':
    """
    moving:是否让机械臂运动
    euler:是否在旋转角度开启阻抗控制
    plot:是否将运动和力用visdom打印
    """
    moving = False
    euler = False
    plot = False

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


    dashboard, move = ConnectRobot()
    dashboard.EnableRobot()
    dashboard.SpeedFactor(60)
    s = force.connect_force()
    

    force_thread = threading.Thread(target=force.get_force, args=(s,))
    force_thread.daemon = True
    force_thread.start()

    if plot:
        record = threading.Thread(target=plotfd)
        record.daemon = True
        record.start()


    sleep(2)

    initial_pose = [138.360397,-472.066620,407.361847,-179.488663,0.264109,179.605057]
    initial_joint = [90.0, 0.0, 100.0, -10.0, -90.0, 0.0]
    print(initial_pose)
    ic = IC(initial_pose=[initial_pose[0] / 1000, initial_pose[1] / 1000, initial_pose[2] / 1000, initial_pose[3], initial_pose[4], initial_pose[5]])
    

    if euler:
        while True:
            start_time = time.time()
            # wrench_external_ = [force[1]/10,-force[2]/3,-force[0]/10,force[4]*10,-force[5]*10,-force[3]*10]
            force_ = [-force.force[1]/10,-force.force[0]/10,-force.force[2]/10,force.force[4]*10,force.force[3]*10,-force.force[5]*10]  #这里将z轴的力设置为旋转轴的力，因为z轴受力没法传给六维力传感器。
            pose, euler = ic.compute_admittance(force_)
            print(initial_pose[0],initial_pose[1],initial_pose[2],euler[0],euler[1],euler[2])
            move.ServoP(initial_pose[0],initial_pose[1],initial_pose[2],euler[0],euler[1],euler[2])
            while time.time() - start_time < 0.01:
                pass
    else:
        while True:
            start_time = time.time()
            # wrench_external_ = [force[1]/10,-force[2]/3,-force[0]/10,force[4]*10,-force[5]*10,-force[3]*10]
            force_ = [-force.force[1]/10,-force.force[0]/10,-force.force[2]/10,force.force[4]*10,force.force[3]*10,-force.force[5]*10]  #这里将z轴的力设置为旋转轴的力，因为z轴受力没法传给六维力传感器。
            pose, euler = ic.compute_admittance(force_)
            print(pose[0]*1000,pose[1]*1000,pose[2]*1000,initial_pose[3],initial_pose[4],initial_pose[5])
            move.ServoP(pose[0]*1000,pose[1]*1000,pose[2]*1000,initial_pose[3],initial_pose[4],initial_pose[5])
            while time.time() - start_time < 0.016:
                pass