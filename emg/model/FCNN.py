import math
import torch
import numpy as np
import pandas as pd
import torch.nn as nn
import torch.optim as optim
from statistics import mean
from torch.utils.data import Dataset, DataLoader

class MyModel(nn.Module):
    # def __init__(self, feature_num, force_num):
    #     super(MyModel, self).__init__()
    #     self.linear1 = nn.Linear(feature_num, 128)
    #     self.dropout_1 = nn.Dropout(0.5)
    #     self.linear2 = nn.Linear(128, 128)
    #     self.dropout_2 = nn.Dropout(0.5)
    #     self.linear3 = nn.Linear(128, force_num)
    #     self.activation = nn.Tanh()

    # def forward(self, x):
    #     x = self.linear1(x)
    #     # x = self.activation(x)
    #     x = self.dropout_1(x)
    #     x = self.linear2(x)
    #     # x = self.activation(x)
    #     x = self.dropout_2(x)
    #     x = self.linear3(x)
    #     #x = torch.softmax(x, dim=1)
    #     return x
    # def __init__(self, feature_num, force_num):
    #     super(MyModel,self).__init__()
    #     print(feature_num)
    #     print(force_num)
    #     self.hidden1=nn.Sequential(
    #             nn.Linear(in_features=feature_num,out_features=128,bias=True),
    #             nn.ReLU())
    #     self.hidden2=nn.Sequential(
    #             nn.Linear(in_features=128,out_features=10,bias=True),
    #             nn.ReLU())
    #     self.hidden3=nn.Sequential(
    #             nn.Linear(in_features=10,out_features=force_num,bias=True),
    #             nn.Sigmoid())
    # def forward(self,x):
    #     fc1=self.hidden1(x)
    #     fc2=self.hidden2(fc1)
    #     output=self.hidden3(fc2)
    #     return output
    def __init__(self, feature_num, force_num):
        super(MyModel, self).__init__()
        self.net=nn.Sequential(
            nn.Linear(in_features=feature_num,out_features=128),nn.LeakyReLU(),
            nn.Linear(128,512),nn.LeakyReLU(),
            nn.Linear(512,128),nn.LeakyReLU(),
            # nn.Linear(1024,128),nn.LeakyReLU(),
            nn.Linear(128,force_num)
        )

    def forward(self, input:torch.FloatTensor):
        return self.net(input)
    

    


class Dst(Dataset):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __len__(self):
        return len(self.x)
    
    def __getitem__(self, i):
        return self.x[i], self.y[i]
    
    
def DatatoTorch(x, y, size, device):
    print(x.shape)
    print(y.shape)
    x = torch.tensor(x).to(device)
    y = torch.tensor(y).to(device)
    train_dst = Dst(x, y)
    loader = DataLoader(train_dst, batch_size=size, shuffle=True)
    return loader


def train(model, loader, device, epoch, name):
    # criterion = nn.L1Loss()
    criterion=nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.0001)
    model.to(device)
    loss_list=[]

    for epoch in range(epoch):
        for i, data in enumerate(loader):
            model.train()  # 将模型设置为训练模式，启用训练相关的操作，例如Dropout或BatchNormalization
            x, y = data
            optimizer.zero_grad()
            outputs = model(x)
            loss = criterion(outputs, y)
            loss.backward()
            loss_list.append(loss.item())
            optimizer.step()

            model.eval()  # 将模型设置为评估模式，禁用训练相关的操作，例如Dropout或BatchNormalization

            # print('Epoch: %d, Loss: %.3f' % (epoch, loss.item()))
        if(epoch%10==0):
            state = {'net':model.state_dict(), 'optimizer':optimizer.state_dict(), 'epoch':epoch}
            torch.save(state, name)
        print('Epoch: %d, Loss: %.3f' % (epoch, mean(loss_list)))
        if(math.isnan(mean(loss_list))):
            print("Something wrong!")
            break
        loss_list=[]
    # torch.save(model.state_dict(), name)
    pass

def result(model, x):
    list=[[]]
    # print(x.shape)
    for i, data in enumerate(x):
        with torch.no_grad():
            outputs = model(torch.unsqueeze(x[i], dim=0)).cpu()
            # print(outputs.size())
            list.append(outputs[0].numpy().tolist())
    data=pd.DataFrame(list)
    return data