import threading
import sys
sys.path.append('./')
from util.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import time
from scipy.spatial.transform import Rotation
import socket
import struct
import argparse
from visdom import Visdom
import util.pytrigno_xhx as pytrigno
import matplotlib.pyplot as plt

from pynput import keyboard


# 全局变量(当前坐标)
current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False
globalLockValue = threading.Lock()

stop_threads = False


def ConnectRobot():
    try:
        # ip = "192.168.5.1"
        ip = "192.168.5.1"
        dashboardPort = 29999
        movePort = 30003
        feedPort = 30004
        print("正在建立连接...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        feed = DobotApi(ip, feedPort)
        print(">.<连接成功>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(连接失败:(")
        raise e

def GetFeed(feed: DobotApi):
    global current_actual
    global algorithm_queue
    global enableStatus_robot
    global robotErrorState
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        feedInfo = np.frombuffer(data, dtype=MyType)
        if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
            globalLockValue.acquire()
            # Refresh Properties
            current_actual = feedInfo["tool_vector_actual"][0]
            algorithm_queue = feedInfo['run_queued_cmd'][0]
            enableStatus_robot=feedInfo['enable_status'][0]
            robotErrorState= feedInfo['error_status'][0]
            globalLockValue.release()
        sleep(0.001)

# admittance parameters
m = [2,2,2,2,2,2]
M_ = np.diag(m)
k = [128,128,100,5,5,5]
K_ = np.diag(k)
# d = [32,25,12,12,12,12]
d = [32,25,12,12,12,12]
D_ = np.diag(d)
arm_max_acc_ = 1

arm_desired_twist_adm_ = np.zeros(6)
arm_desired_pose_ = np.zeros(6)

wrench_external_ = np.zeros(6)

desired_pose_position_ = np.array([138.360397/1000,-472.066620/1000,407.361847/1000]) 
desired_pose_orientation_ = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])
desired_pose_orientation_euler_ = np.array([179.488663,0.264109,179.605057]) 
arm_position_ = np.array([138.360397/1000,-472.066620/1000,407.361847/1000]) 
arm_orientation_ = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])
arm_orientation_euler_ = np.array([179.488663,0.264109,179.605057]) 
error = np.zeros(6)

# 计算循环的时间间隔
# loop_rate = 20  # Hz
loop_rate = 75 # Hz
sec = 1.0 / loop_rate
i = 1

def compute_admittance(moving = False):
    global arm_desired_twist_adm_, arm_desired_pose_, arm_position_, arm_orientation_euler_,i
    if moving:
        if desired_pose_position_[1]>-0.38:
            i = -1
        if desired_pose_position_[1]<-0.6:
            i = 1
        desired_pose_position_[1] = desired_pose_position_[1] + 200/100000*i

    error[0:3] = arm_position_ - desired_pose_position_
    error[3:] = arm_orientation_euler_ - desired_pose_orientation_euler_
    coupling_wrench_arm = np.dot(D_, arm_desired_twist_adm_) + np.dot(K_, error)
    arm_desired_accelaration = np.linalg.inv(M_) @ (-coupling_wrench_arm + wrench_external_)
    a_acc_norm = np.linalg.norm(arm_desired_accelaration[0:3])
    if a_acc_norm > arm_max_acc_:
        # print("Admittance generates high arm acceleration! norm:", a_acc_norm)
        arm_desired_accelaration[0:3] *= (arm_max_acc_ / a_acc_norm)
    arm_desired_twist_adm_ += arm_desired_accelaration * sec  #进行速度迭代并记录
    arm_desired_pose_ = arm_desired_twist_adm_ * sec  #这里应该用arm_desired_twist_adm_+当前速度
    arm_position_ += arm_desired_pose_[0:3] 
    arm_orientation_euler_ += arm_desired_pose_[3:6] 
    return arm_desired_pose_, arm_orientation_euler_


PORT = 49152  # Ethernet DAQ使用的端口号
SAMPLE_COUNT = 100000  # 10个输入样本
SPEED = 10  # 1000 / SPEED = 频率（单位：赫兹）;  //0-255，频率通过 1000/num 计算
FILTER = 4  # 0 = 无滤波; 1 = 500赫兹; 2 = 150赫兹; 3 = 50赫兹; 4 = 15赫兹; 5 = 5赫兹; 6 = 1.5赫兹
BIASING_ON = 0xFF  # 开启偏置
BIASING_OFF = 0x00  # 关闭偏置
COMMAND_START = 0x0002  # Command for start streaming
COMMAND_STOP = 0x0000  # Command for stop streaming
COMMAND_BIAS = 0x0042  # Command for toggle biasing
COMMAND_FILTER = 0x0081  # Command for setting filter
COMMAND_SPEED = 0x0082  # Command for setting speed
force = []


def connect_force():
    s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.connect(('192.168.2.3',49152))
    send_command(s, COMMAND_SPEED, SPEED)
    send_command(s, COMMAND_FILTER, FILTER)
    send_command(s, COMMAND_BIAS, BIASING_ON)
    send_command(s, COMMAND_START, SAMPLE_COUNT)
    return s

def send_command(sock, command, data):
    request = bytearray(8)
    struct.pack_into('!H', request, 0, 0x1234)
    struct.pack_into('!H', request, 2, command)
    struct.pack_into('!L', request, 4, data)
    sock.send(request)
    time.sleep(0.005)  # Wait a little, assuming the command has been processed by the Ethernet DAQ

#处理数据
def parse_data(inBuffer):
    FORCE_DIV = 10000.0 
    cur = list(range(6))
    uItems = 0
    cur[0] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
    uItems += 1
    cur[1] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
    uItems += 1
    cur[2] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
    uItems += 1
    cur[3] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
    uItems += 1
    cur[4] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
    uItems += 1
    cur[5] = struct.unpack('>i', inBuffer[12 + (uItems * 4):16 + (uItems * 4)])[0]/FORCE_DIV
    return cur


def filt():
    global force
    for i in range(0,len(force)):
        if force[i]<1 and force[i]>-1:
            force[i] = 0

def get_force(s):
    global force
    while True:
        inBuffer = s.recv(36)
        force = parse_data(inBuffer)
        filt()

def plot_viz():
    global force
    force_list = []
    pose_list = []
    viz = Visdom()
    opts1 = {
        "title": 'FORCE',
        # "xlabel":'x',
        # "ylabel":'y',
        "width":750,
        "height":300,
        "legend":['f1','f2','f3','f4','f5','f6'],
        "ylim":[0, 50]
    }
    opts2 = {
        "title": 'POSE',
        # "xlabel":'x',
        # "ylabel":'y',
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


def emg():
    global K_,D_,M_
    channel=1
    dev_emg = pytrigno.TrignoEMG(channel_range=(0,channel-1), samples_per_read=500,
                        host='127.0.0.1')
    dev_emg.cmd_conn()
    dev_emg.start()
    data_EMG = dev_emg.read()
    while True:
        if stop_threads == 1:
            break
        data_EMG = dev_emg.read()
        data_EMG = np.abs(data_EMG)
        data_EMG = np.mean(data_EMG)
        # print(data_EMG)
        if data_EMG<0.00005:
            m = [2,2,2,2,2,2]
            M_ = np.diag(m)
            k = [100,100,100,5,5,5]
            K_ = np.diag(k)
            d = [32,32,32,2,2,2]
            D_ = np.diag(d)
        else:
            # print(1)
            m = [10,10,10,2,2,2]
            M_ = np.diag(m)
            k = [2000,2000,2000,5,5,5]
            K_ = np.diag(k)


data_ACCL = []
dt = 1/148

vx = 0
vy = 0
vz = 0

x = 0
y = 0
z = 0

def accl():
    global data_ACCL, vx, vy, vz, x, y, z
    channel=16
    steps = 1400
    dev_emg = pytrigno.TrignoAccel(channel_range=((channel-1)*3,channel*3-1), samples_per_read=steps,
                        host='127.0.0.1')
    dev_emg.cmd_conn()
    dev_emg.start()
    data_ACCL = dev_emg.read()

    # last = 0

    # while True:
    #     if stop_threads == 1:
    #         break
    #     data_ACCL = dev_emg.read()
    #     data_ACCL = np.average(data_ACCL, axis=1) 
    #     if data_ACCL[0]**2+data_ACCL[1]**2+(data_ACCL[2]-1)**2<0.01:
    #         vx=0
    #         vy=0
    #         vz=0
    #     else:
    #         vx += data_ACCL[0]*dt*steps*9.8/2
    #         vy += data_ACCL[1]*dt*steps*9.8/2
    #         vz += (data_ACCL[2]-1)*dt*steps*9.8/2

        # if abs(data_ACCL[0])<0.01:
        #     data_ACCL[0]=0
        # if abs(data_ACCL[1])<0.01:
        #     data_ACCL[1]=0
        # if abs(data_ACCL[2])<0.01:
        #     data_ACCL[2]=0
        # # print(data_ACCL)

        # cur=vx**2+vy**2+vz**2
        # if abs(cur-last)<0.1:
        #     vx=0
        #     vy=0
        #     vz=0
        # last=vx**2+vy**2+vz**2

        # x += vx*dt*steps
        # y += vy*dt*steps
        # z += vz*dt*steps
        # print(vx,vy,vz)
        # print(x,y,z)
        # data_ACCL[0]
    
def accl1():
    global data_ACCL, vx, vy, vz, x, y, z
    channel=16
    steps = 30
    dev_emg = pytrigno.TrignoAccel(channel_range=((channel-1)*3,channel*3-1), samples_per_read=steps,
                        host='127.0.0.1')
    dev_emg.cmd_conn()
    dev_emg.start()
    data_ACCL = dev_emg.read()

    # last = 0

    while True:
        if stop_threads == 1:
            break
        data_ACCL = dev_emg.read()
        data_ACCL[2] = [x -0.9788 for x in data_ACCL[2]]
        data_ACCL[0] = [0 if abs(x) < 0.08 else x for x in data_ACCL[0]]
        data_ACCL[1] = [0 if abs(x) < 0.08 else x for x in data_ACCL[1]]
        data_ACCL[2] = [0 if abs(x) < 0.08 else x for x in data_ACCL[2]]
        # print(data_ACCL)
        # data_ACCL = np.average(data_ACCL, axis=1)
        if(all(x == 0 for x in data_ACCL[0])):
            vx=0
        else:
            xx = np.average(data_ACCL[0])
            vx += xx*dt*steps*9.8/2
        if(all(x == 0 for x in data_ACCL[1])):
            vy=0
        else:
            yy = np.average(data_ACCL[1])
            vy += yy*dt*steps*9.8/2
        if(all(x == 0 for x in data_ACCL[2])):
            vz=0
        else:
            zz = np.average(data_ACCL[2])
            vz += zz*dt*steps*9.8/2



        # if data_ACCL[0]**2+data_ACCL[1]**2+(data_ACCL[2]-0.9788)**2<0.01:
        #     vx=0
        #     vy=0
        #     vz=0
        # else:
        #     vx += data_ACCL[0]*dt*steps*9.8/2
        #     vy += data_ACCL[1]*dt*steps*9.8/2
        #     vz += (data_ACCL[2]-1)*dt*steps*9.8/2

        # if abs(data_ACCL[0])<0.01:
        #     data_ACCL[0]=0
        # if abs(data_ACCL[1])<0.01:
        #     data_ACCL[1]=0
        # if abs(data_ACCL[2])<0.01:
        #     data_ACCL[2]=0
        # # print(data_ACCL)

        # cur=vx**2+vy**2+vz**2
        # if abs(cur-last)<0.1:
        #     vx=0
        #     vy=0
        #     vz=0
        # last=vx**2+vy**2+vz**2

        # x += vx*dt*steps
        # y += vy*dt*steps
        # z += vz*dt*steps
        # print(vx,vy,vz)
        # print(x,y,z)
        # data_ACCL[0]
    


# 键盘按下执行的函数 使用try和except的原因是有特殊按键（功能键）
def keyboard_on_press(key):
    global stop_threads
    try:
        print('字母键{0} press'.format(key.char))
    except AttributeError:
        print('特殊键{0} press'.format(key))
        if key == keyboard.Key.esc:
            stop_threads = True
            return False
        
def control():
    global vx,vy,vz
    while True:
        if stop_threads == 1:
            break
        print(vx,vy,vz)
        sleep(0.01)


       
if __name__ == '__main__':


    listener = keyboard.Listener(on_press=keyboard_on_press)
    listener.daemon = 1
    listener.start()

    # emg_thread = threading.Thread(target=emg)
    # emg_thread.daemon = True
    # emg_thread.start()
    # sleep(1)

    accl_thread = threading.Thread(target=accl1)
    accl_thread.daemon = True
    accl_thread.start()

    control_thread = threading.Thread(target=control)
    control_thread.daemon = True
    control_thread.start()

    # sleep(18)
    # print(data_ACCL)
    # steps=0.01
    # vz=0
    # pz=0
    # vz_list=[]
    # pz_list=[]
    # for i in range(30,1400):
    #     # vz += (data_ACCL[1][i]-0.99501026)*steps*9.8
    #     if abs(data_ACCL[0][i])<0.02:
    #         data_ACCL[0][i]=0
    #     vz += data_ACCL[0][i]*steps*9.8
    #     pz += vz*steps
    #     vz_list.append(vz)
    #     pz_list.append(pz)
    # plt.plot(data_ACCL[0], label="loss")
    # plt.plot(vz_list, label="loss")
    # # plt.plot(pz_list, label="loss")
    # plt.legend()
    # plt.show()



    while True:
        if stop_threads:
            break
        # print(1)
        pass