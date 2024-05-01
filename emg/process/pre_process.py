import os
import numpy as np
import pandas as pd
from scipy import signal
from scipy.io import loadmat
from sklearn.preprocessing import StandardScaler

class Data():
    def __init__(self, emg_channel, force_channel):
        self.emg_raw = []
        self.force_raw = []
        self.emg_signal = pd.DataFrame()
        self.force_signal = pd.DataFrame()
        self.emg_channel = emg_channel
        self.force_channel = force_channel
        self.data_num = 0

    def get_data(self, path, file):
        mat = loadmat(os.path.join(path,file))
        self.emg_raw = self.emg_signal = pd.DataFrame(mat['emg'])
        self.force_raw = self.force_signal = pd.DataFrame(mat['force'])
        self.data_num = len(self.emg_signal)-1
        # return data,force

    def crop_data(self, path, file):  #对数据进行裁剪，针对情况自己写
        mat = loadmat(os.path.join(path,file))
        all = pd.concat([self.emg_signal,self.force_signal],axis=1)
        all['stimulus'] = mat['restimulus']
        all['repetition'] = mat['repetition']
        reps = [5,6]
        gestures = [41,42,43,44,45,46]
        if reps:
            x = [np.where(all.values[:,-1] == rep) for rep in reps]
            indices = np.squeeze(np.concatenate(x, axis = -1))
            all = all.iloc[indices, :]
            all = all.reset_index(drop=True)

        if gestures:
            x = [np.where(all.values[:,-2] == move) for move in gestures]
            indices = np.squeeze(np.concatenate(x, axis = -1))
            all = all.iloc[indices, :]
            all = all.reset_index(drop=True)

        self.emg_signal = all.iloc[:,:12]
        self.force_signal = all.iloc[:,12:18]
        #返回一个字典，用于参数记录
        return {'reps':reps, 'gestures':gestures}
    

    def normalise(self):  #!没写完，标准化变成均值为0
        print("Not yet!!!!!!!!!!!!!!!!!!!!")
        
        scaler = StandardScaler(with_mean=True,
                                    with_std=True,
                                    copy=False).fit(self.emg_signal.iloc[:, :])
        
        scaled = scaler.transform(self.emg_signal.iloc[:,:])
        self.emg_signal = pd.DataFrame(scaled)
    

    def filter_data(self, f, butterworth_order = 4, btype = 'lowpass'):
        #力并没有进行滤波，因为后面窗口内的取均值作为真值
        emg_data = self.emg_signal.values[:,:]
   
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
        
        self.emg_signal = pd.DataFrame(transpose.T)

    def rectify_data(self):
        self.emg_signal = abs(self.emg_signal)

    def windowing_data(self, win_len, win_stride):

        idx=  [i for i in range(win_len, len(self.emg_signal), win_stride)]
        
        x = np.zeros([len(idx), win_len, self.emg_channel])
        force_win=np.zeros([win_len, self.force_channel])
        y = np.zeros([len(idx), self.force_channel])
        
        for i,end in enumerate(idx):
            start = end - win_len
            x[i] = self.emg_signal.iloc[start:end, :].values
            force_win = self.force_signal.iloc[start:end, :].values
            y[i] = np.average(force_win,axis=0)
        return x, y
    
    def windowing_data2(self, win_len, win_stride, len_data):

        idx=  [i for i in range(win_len, len_data, win_stride)]
        
        x = np.zeros([len(idx), win_len, self.emg_channel])
        force_win=np.zeros([1, self.force_channel])
        y = np.zeros([len(idx), self.force_channel])
        
        #别忘了这里有20的窗口重叠
        for i,end in enumerate(idx):
            start = end - win_len
            x[i] = self.emg_signal.iloc[start:end, :].values
            # 这里将numpy变换成和K大小基本一致的数
            # x[i] = np.multiply(self.emg_signal.iloc[start:end, :].values,100)
            force_win = self.force_signal.iloc[i+1,:self.force_channel].values  #这里写成i+1相当于用前200个肌电来进行预测
            # y[i] = force_win
            y[i] = np.absolute(np.multiply(force_win,0.01))
            # print(x[i],y[i])
        return x, y
    
# data=Data(12,6)
# data.get_data('/remote-home/2230728/project/EMG/NinaPro/DB2', 'S5_E3_A1.mat')
# data.emg_signal[:100000].plot(figsize = (15,10))
# data.filter_data(f=20, butterworth_order=4, btype='lowpass')
# data.emg_signal[:100000].plot(figsize = (15,10))
# data.rectify_data()
# data.emg_signal[:100000].plot(figsize = (15,10))
# data.windowing_data(200, 100)