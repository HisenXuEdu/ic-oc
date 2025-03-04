import os
import sys
import time
import argparse
import numpy as np
import pandas as pd

from scipy import signal
from sklearn.preprocessing import StandardScaler


# 将父级目录加入到import的path中
current_dir = os.path.dirname(__file__)
print(current_dir)
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
sys.path.append(current_dir)

import pytrigno as pytrigno


# from process.pre_process import *
from process.feature import *
from model.LSTM import *
import torch


class Emg:
    def __init__(self, model, channel=1, host='127.0.0.1'):
        
        self.model = MyModel(8, 3)
        checkpoint = torch.load(model)
        self.model.load_state_dict(checkpoint['net'])
        self.K=[0,0,0]

        self.channel = channel
        self.dev_emg = pytrigno.TrignoEMG(channel_range=(0,self.channel-1), samples_per_read=400,
                    host=host)
        self.dev_emg.start()
        # data=Data(6,3)
        # self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # print(self.device)
        # self.model.to(self.device)

        
    def get_single(self):
        x = self.dev_emg.read()
        self.x = pd.DataFrame(x.T)
        # print(self.x)
        self.normalise()
        self.filter_data(f=(20,50), butterworth_order=4, btype='bandpass')
        self.rectify_data()
        x = self.windowing(200,20,400)
        feature = Feature(x)
        feature.time_features_estimation(x, 200)
        x = feature.time_features_matrix.astype(np.float32)
        x = torch.tensor(x)
        return x
    
    def get_K(self):
        x = self.get_single()
        # print(x)
        res = result(self.model, x)
        self.K=res
        return res
    
    def normalise(self):  #!没写完，标准化变成均值为0
        print("Not yet!!!!!!!!!!!!!!!!!!!!")
        
        scaler = StandardScaler(with_mean=True,
                                    with_std=True,
                                    copy=False).fit(self.x.iloc[:, :])
        
        scaled = scaler.transform(self.x.iloc[:,:])
        self.x = pd.DataFrame(scaled)
    
    def filter_data(self, f, butterworth_order = 4, btype = 'lowpass'):
        #力并没有进行滤波，因为后面窗口内的取均值作为真值
        emg_data = self.x.values[:,:]
   
        f_sampling = 2000
        nyquist = f_sampling/2
        if isinstance(f, int):
            fc = f/nyquist
        else:
            fc = list(f)
            for i in range(len(f)):
                fc[i] = fc[i]/nyquist
                
        b,a = signal.butter(butterworth_order, fc, btype=btype)
        transpose = emg_data.T.copy()
        
        for i in range(len(transpose)):
            transpose[i] = (signal.lfilter(b, a, transpose[i]))
        
        self.x = pd.DataFrame(transpose.T)
    
    def rectify_data(self):
        self.x = abs(self.x)

    def windowing(self, win_len, win_stride, len_data):

        idx=  [i for i in range(win_len, len_data, win_stride)]
        x = np.zeros([len(idx), win_len, self.channel])
        
        #别忘了这里有20的窗口重叠
        for i,end in enumerate(idx):
            start = end - win_len
            x[i] = self.x.iloc[start:end, :].values
        return x

    def stop(self):
        self.dev_emg.stop()


class Emg_S:
    def __init__(self, channel=1, host='127.0.0.1'):

        self.K=[0,0,0]

        self.channel = channel
        self.dev_emg = pytrigno.TrignoEMG(channel_range=(0,self.channel-1), samples_per_read=200,
                    host=host)
        self.dev_emg.start()

        
    def get_single(self):
        x = self.dev_emg.read()
        self.x = pd.DataFrame(x.T)
        x=np.abs(x)
        x=np.mean(x,axis=1)
        # print(self.x)
        # self.normalise()
        self.filter_data(f=(20,50), butterworth_order=4, btype='bandpass')
        self.rectify_data()
        data_EMG = np.array(self.x)
        data_EMG = np.abs(data_EMG)
        data_EMG = np.mean(data_EMG)
        return x
    
    
    def get_single_network(self):
        x = self.dev_emg.read()
        self.x = pd.DataFrame(x.T)
        x=np.abs(x)
        x=np.mean(x,axis=1)
        # print(self.x)
        # self.normalise()
        self.filter_data(f=(20,50), butterworth_order=4, btype='bandpass')
        self.rectify_data()
        self.envelope_data()
        # data_EMG = np.array(self.x)
        # data_EMG = np.abs(data_EMG)
        # data_EMG = np.mean(data_EMG)
        return x
    
    #包络
    def envelope_data(self, window = 200):
        self.x = pd.DataFrame(self.x).rolling(window=window, center=False).mean()
        self.x = self.x.dropna()
    
    def get_K(self):
        x = self.get_single()
        # print(x)
        res = result(self.model, x)
        self.K=res
        return res
    
    def normalise(self):  #!没写完，标准化变成均值为0
        
        scaler = StandardScaler(with_mean=True,
                                    with_std=True,
                                    copy=False).fit(self.x.iloc[:, :])
        
        scaled = scaler.transform(self.x.iloc[:,:])
        self.x = pd.DataFrame(scaled)
    
    def filter_data(self, f, butterworth_order = 4, btype = 'lowpass'):
        #力并没有进行滤波，因为后面窗口内的取均值作为真值
        emg_data = self.x.values[:,:]
   
        f_sampling = 2000
        nyquist = f_sampling/2
        if isinstance(f, int):
            fc = f/nyquist
        else:
            fc = list(f)
            for i in range(len(f)):
                fc[i] = fc[i]/nyquist
                
        b,a = signal.butter(butterworth_order, fc, btype=btype)
        transpose = emg_data.T.copy()
        
        for i in range(len(transpose)):
            transpose[i] = (signal.lfilter(b, a, transpose[i]))
        
        self.x = pd.DataFrame(transpose.T)
    
    def rectify_data(self):
        self.x = abs(self.x)

    def windowing(self, win_len, win_stride, len_data):

        idx=  [i for i in range(win_len, len_data, win_stride)]
        x = np.zeros([len(idx), win_len, self.channel])
        
        #别忘了这里有20的窗口重叠
        for i,end in enumerate(idx):
            start = end - win_len
            x[i] = self.x.iloc[start:end, :].values
        return x

    def stop(self):
        self.dev_emg.stop()

if __name__ == '__main__':
    # emg = Emg(model='model_path/i8o3.pth', channel=6, host='127.0.0.1')
    # while(1):
    #     k = emg.get_K()
    #     print(k.drop(k.index[[0]]).mean(axis=0))
    # emg.stop()

    emg = Emg_S(channel=3, host='127.0.0.1')
    while(1):
        cur = emg.get_single()
        cur[0] =1000 - (cur[0]*10000000-400)/2.5
        print(cur)
    emg.stop()

