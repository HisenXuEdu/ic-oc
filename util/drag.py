import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import time
import numpy as np
import re
import random
import pandas as pd
import datetime
import multiprocessing
import os
from pynput import keyboard



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
    os.system('python ./pose_record.py')

def keyboard_on_press(key):
    global stop_threads
    try:
        print('字母键{0} press'.format(key.char))
    except AttributeError:
        print('特殊键{0} press'.format(key))
        if key == keyboard.Key.esc:
            stop_threads = True
            return False


if __name__ == '__main__':
    stop_threads=False
    dashboard, move = ConnectRobot()
    dashboard.EnableRobot()

    listener = keyboard.Listener(on_press=keyboard_on_press)
    listener.daemon = 1
    listener.start()

    dashboard.SpeedFactor(60)
    dashboard.StopDrag()

    dashboard.StartDrag()
    while(not stop_threads):
        sleep(0.3)
    dashboard.StopDrag()