import sys
from util.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
import numpy as np
from time import sleep
import pandas as pd
from visdom import Visdom


opt_pose  = {
            "title": 'POSE',
            "width":1000,
            "height":500,
            "legend":['x','y','z','r','p','y']
            }
opt_force = {
            "title": 'FORCE',
            "width":1000,
            "height":500,
            "legend":['f1','f2','f3','f4','f5','f6'],
            "ylim":[0, 50]
            }
opt_circle = {
            "title": 'DATA',
            "width":1500,
            "height":600,
            "legend":['f1','f2','f3','f4','f5','f6'],
            "ylim":[0, 50],
            "markerize":50,
            "markers":True
            }


class Plot():
    def __init__(self,win_size=200,title='DATA',opt={}):
        self.viz=Visdom()
        self.data_list=[]
        self.win=win_size
        self.title=title
        if not opt:
            self.opt={
            "title": self.title,
            "width":1000,
            "height":500,
            "legend":['x','y','z','r','p','y']
        }
        else:
            self.opt=opt
        self.opt["title"]=self.title



    def plot(self,data):
        self.data_list.append(data)
        sleep(0.01)
        if len(self.data_list)>self.win:
            self.data_list.pop(0)
        self.viz.line(Y=self.data_list, win=self.title, opts=self.opt)
        pass