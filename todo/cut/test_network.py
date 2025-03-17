from model.LSTM import *
from emg.emg import *
import numpy as np
import torch
import matplotlib.pyplot as plt

def change_para_emg():
        global emg, ic
        while True:
            x = generate()
            # 对x转置
            x = x.T
            x = x.astype(np.float32)
            # x = np.zeros([1, win_len, 6])
            # x = x.T
            # 求x第一列的均值
            mean_first_column = np.mean(x[:, 0])
            print("Mean of the first column:", mean_first_column)
            print(x.shape)
            x = torch.tensor(x).to(device)
            res = result_realtime(model, x)
            print(res)

def change_para_emg1():
        global emg, ic
        k = []
        # 运行10s
        for i in range(100):
            data_EMG = emg.get_single_network()
            # 对x转置
            print(data_EMG)
            # 将data_EMG第二个维度降采样变成20个
            data_EMG = data_EMG[:, ::data_EMG.shape[1] // 20]
            if data_EMG.shape[1] > 20:
                data_EMG = data_EMG[:, :20]
            x = []
            data_EMG = data_EMG/0.0002
            
            # for i in range(2):
            #     x.append(np.random.rand(20))
            # x.append(data_EMG[0])
            # for i in range(3):
            #     x.append(np.random.rand(20))

            x.append(data_EMG[0])
            x.append(data_EMG[0])
            for i in range(4):
                x.append(np.zeros(20))
            # x.append(data_EMG[0])
            # for i in range(4):
            #     x.append(np.zeros(20))
            
            # for i in range(6):
            #     x.append(np.zeros(20))

            x = np.array(x)
            x = x.T
            x = x.astype(np.float32)
            # x = np.zeros([1, win_len, 6])
            # x = x.T
            # 求x第一列的均值
            mean_first_column = np.mean(x[:, 0])
            print("Mean of the first column:", mean_first_column)
            print(x.shape)
            x = torch.tensor(x).to(device)
            res = result_realtime(model, x)
            print(res)
            k.append(res)
        return k

def generate():
     # 生成六维的x， 每个维度生成一个长度为200的随机序列
    x = []
    for i in range(6):
        x.append(np.random.rand(20))
    x = np.array(x)
    return x
    
win_len = 20
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
emg = Emg_S(channel=1, host='127.0.0.1')
model = MyModel(6, win_len,1)
model.to(device)
checkpoint = torch.load('model_path/LSTMi1o1.pth', map_location=torch.device('cpu'))
model.load_state_dict(checkpoint['net'])
x = generate()
k = change_para_emg1()

# 绘制K
k = np.array(k)
plt.plot(k[:, 0], label='kx')
plt.legend()
plt.show()

