from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
import numpy as np
import time
from time import sleep
import pandas as pd
from visdom import Visdom
import threading
import sys


    """主要是一些工具函数

    """

def str2bool(v):
    """解析传入的参数是否为true
    """
    return v.lower() in ("True", "true", "T", "t", "yes", "1")
    
def ConnectRobot():
    try:
        ip = "192.168.5.1"
        feedPort = 30004
        # print("正在建立连接...")
        feed = DobotApi(ip, feedPort)
        # print(">.<pose_record连接成功>!<")
        return feed
    except Exception as e:
        print(":(pose_record连接失败:(")
        raise e
    
def plotp():
    global pose_list
    pose = []
    viz = Visdom()
    opts1 = {
        "title": 'POSE',
        "width":1500,
        "height":600,
        "legend":['f1','f2','f3','f4','f5','f6'],
        "ylim":[0, 50],
        "markerize":50,
        "markers":True
    }
    while True:
        sleep(0.05)
        pose.append([pose_list[-1][0]-138.360397, pose_list[-1][1]+472.066620, pose_list[-1][2]-407.361847])
        if len(pose_list)>500:
            pose.pop(0)
        viz.line(Y=pose, win='pose', opts=opts1)

def plotfp(pose, force):
    """用visdom打印轨迹和力
    这种方式不太好，因为需要在主线程中进行调用
    Args:
        pose (np.ndarray): 六维位置
        force (np.ndarray): 六维力
    """
    force_list = []
    pose_list = []
    viz = Visdom()
    opts1 = {
        "title": 'FORCE',
        "width":750,
        "height":300,
        "legend":['f1','f2','f3','f4','f5','f6'],
        "ylim":[0, 50]
    }
    opts2 = {
        "title": 'POSE',
        "width":750,
        "height":300,
        "legend":['x','y','z','r','p','y']
        
    }
    while True:
        sleep(0.01)
        force_list.append([-force[1],-force[0],-force[2]])
        # force_list.append([force[4],force[3],-force[5]])
        pose_list.append([current_actual[0]-138.360397,current_actual[1]+472.066620,current_actual[2]-407.361847])
        # pose_list.append([current_actual[3]-179.488663,current_actual[4]+0.264109,current_actual[5]-179.605057])
        if len(pose_list)>200:
            force_list.pop(0)
            pose_list.pop(0)
        viz.line(Y=force_list, win='force', opts=opts1)
        viz.line(Y=pose_list, win='pose', opts=opts2)


#传递参数分别为record和plot
if __name__ == '__main__':

    # 第一个元素是脚本本身的名称，因此我们忽略它
    write = False
    plot = False

    args = sys.argv[1:]
    if len(args) == 1:
        write = str2bool(args[0])
    elif len(args) == 2:
        write = str2bool(args[0])
        plot = str2bool(args[1])

    
    print("write:",write,"    ","plot:",plot)

    feed = ConnectRobot()
    timestamp_list = []
    pose_list = []
    hasRead = 0
    timestamp = 0
    first_time = 0
    first_feed = True

    if plot:
        viz = threading.Thread(target=plotp)
        viz.daemon = True
        viz.start()

    t_start = time.time()
    t_start_string = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_start))
    print("位置开始时间：", t_start_string)

    while timestamp - first_time < 30000:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        feedInfo = np.frombuffer(data, dtype=MyType)

        if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
            current_actual = feedInfo["tool_vector_actual"][0]
            algorithm_queue = feedInfo['run_queued_cmd'][0]
            enableStatus_robot=feedInfo['enable_status'][0]
            robotErrorState= feedInfo['error_status'][0]
            timestamp = feedInfo['time_stamp'][0]
            pose = feedInfo['tool_vector_actual'][0]
            if first_feed:
                first_feed = False
                t_start = time.time()
                t_start_string = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_start))
                # print(t_start_string)
                # print(int(t_start*1000))
                first_time = timestamp
                # t_robot = first_time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_start))
                # print(t_robot)
            timestamp_list.append(timestamp)
            pose_list.append(pose)
    if write:        
        t_end = time.time()
        t_end_string = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_end))
        print("位置结束时间：", t_end_string)
        data = pd.DataFrame(pose_list, columns=None)
        # data['time'] = pd.DataFrame(timestamp_list)
        data.to_csv('./data/pose/POSE'+ str(int(t_start*1000)) +'.csv', index=None)