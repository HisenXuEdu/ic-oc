import os
import sys
import time
import argparse
import numpy as np
import pandas as pd

# 将父级目录加入到import的path中
current_dir = os.path.dirname(__file__)
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
import emg.pytrigno as pytrigno


from process.pre_process import *
from process.feature import *
from model.FCNN import *
import torch


class Emg:
    def __init__(self, mdoel, channel=1, host='127.0.0.1'):
        self.channel = channel
        self.dev_emg = pytrigno.TrignoEMG(channel_range=(0,self.channel-1), samples_per_read=400,
                    host=host)
        self.dev_emg.start()
        # data=Data(6,3)
        # self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # print(self.device)
        self.model = MyModel(8, 3)
        # self.model.to(self.device)
        checkpoint = torch.load('model_path/FCNN-b32-29:13:09-i8o3.pth')
        self.model.load_state_dict(checkpoint['net'])
        self.K=[0,0,0]
        
    def get_single(self):
        self.x = self.dev_emg.read()
        self.normalise()
        self.filter_data(f=(20,50), butterworth_order=4, btype='bandpass')
        self.rectify_data()
        feature(x)
        feature.time_features_estimation(x, 200)
        x = feature.time_features_matrix.astype(np.float32)
        x = torch.tensor(x)
        return x
    
    def get_K(self):
        x = self.get_single()
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


if __name__ == '__main__':
    emg = Emg()
    while(1):
        k = emg.get_K()
        print(k)

