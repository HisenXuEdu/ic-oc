import sys
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
import numpy as np
import time
from time import sleep
import pandas as pd
from visdom import Visdom
import threading


class Plot():
    def __init__(self,win_size=200,title='data'):
        self.viz=Visdom()
        self.data_list=[]
        self.win=win_size
        self.title=title
        self.opts1 = {
            "title": self.title,
            "width":750,
            "height":300,
            "legend":['f1','f2','f3','f4','f5','f6'],
            "ylim":[0, 50]
        }
        self.opts2 = {
            "title": self.title,
            "width":750,
            "height":300,
            "legend":['x','y','z','r','p','y']
            
        }

    def plot(self,data):
        self.data_list.append(data)
        sleep(0.01)
        if len(self.data_list)>self.win:
            self.data_list.pop(0)
        self.viz.line(Y=self.data_list, win=self.title, opts=self.opts1)
        pass