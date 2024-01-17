import sys
import os

# 将父级目录加入到import的path中
current_dir = os.path.dirname(__file__)
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import threading
from util.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re
from scipy.spatial.transform import Rotation

globalLockValue = threading.Lock()
enableStatus_robot = None
algorithm_queue = None

def ClearRobotError(dashboard: DobotApiDashboard):
    global robotErrorState
    dataController,dataServo =alarmAlarmJsonFile()    # 读取控制器和伺服告警码
    if True:
                numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
                numbers= [int(num) for num in numbers]
                if (numbers[0] == 0):
                  if (len(numbers)>1):
                    for i in numbers[1:]:
                      alarmState=False
                      if i==-2:
                          print("机器告警 机器碰撞 ",i)
                          alarmState=True
                      if alarmState:
                          continue                
                      for item in dataController:
                        if  i==item["id"]:
                            print("机器告警 Controller errorid",i,item["zh_CN"]["description"])
                            alarmState=True
                            break 
                      if alarmState:
                          continue
                      for item in dataServo:
                        if  i==item["id"]:
                            print("机器告警 Servo errorid",i,item["zh_CN"]["description"])
                            break  
                       
                    choose = input("输入1, 将清除错误, 机器继续运行: ")     
                    if  int(choose)==1:
                        dashboard.ClearError()
                        sleep(0.01)
                        dashboard.Continue()


if __name__ == '__main__':
    try:
        ip = "192.168.5.1"
        dashboardPort = 29999
        print("正在建立连接...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        print(">.<连接成功>!<")
    except Exception as e:
        print(":(连接失败:(")
        raise e
    dashboard.ClearError()
    sleep(0.01)
    dashboard.Continue()
    ClearRobotError(dashboard)
