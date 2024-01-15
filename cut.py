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
    dashboard, move = ConnectRobot()
    dashboard.EnableRobot()
    dashboard.SpeedFactor(80)

    s = force.connect_force()
    

    force_thread = threading.Thread(target=force.get_force, args=(s,))
    force_thread.daemon = True
    force_thread.start()

    sleep(2)

    initial_pose = [141.932007,-541.146973,391.485504,90.798767,0.044857,0.014894]
    print(initial_pose)
    move.MovL(initial_pose[0],initial_pose[1],initial_pose[2],initial_pose[3],initial_pose[4],initial_pose[5])
    sleep(6)

    ic = IC(initial_pose =[initial_pose[0]/1000,initial_pose[1]/1000,initial_pose[2]/1000,initial_pose[3],initial_pose[4],initial_pose[5]])
    while True:
        start_time = time.time()
        # wrench_external_ = [force[1]/10,-force[2]/3,-force[0]/10,force[4]*10,-force[5]*10,-force[3]*10]
        force_ = [force.force[1]/10,-force.force[2]/3,-force.force[4],force.force[4]*10,-force.force[5]*10,-force.force[3]*10]  #这里将z轴的力设置为旋转轴的力，因为z轴受力没法传给六维力传感器。
        pose, euler = ic.compute_admittance(force_)
        print(pose[0]*1000,pose[1]*1000,pose[2]*1000,90.798767,0.044857,0.014894)
        move.ServoP(pose[0]*1000,pose[1]*1000,initial_pose[2],euler[0],0.044857,0.014894)
        while time.time() - start_time < 0.016:
            pass