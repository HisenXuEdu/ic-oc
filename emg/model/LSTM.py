import math
import torch
import numpy as np
import torch.nn as nn
import torch.optim as optim
import pandas as pd
from statistics import mean
from torch.utils.data import Dataset, DataLoader

class MyModel(nn.Module):
    def __init__(self, nodes, win_len, output_num):
        super(MyModel, self).__init__()
        self.lstm_1 = nn.LSTM(nodes, nodes, num_layers=2, dropout=0.2, batch_first=True)
        self.lstm_2 = nn.LSTM(nodes, nodes, num_layers=2,dropout=0.2, batch_first=True)
        # self.lstm_3 = nn.LSTM(nodes, nodes, dropout=0.2, batch_first=True)
        # self.lstm_4 = nn.LSTM(nodes, nodes, dropout=0.2, batch_first=True)   #输出是100*12
        self.flatten = nn.Flatten()   #压缩成1200
        self.dense_1 = nn.Linear(nodes*win_len, 128)
        self.dropout_1 = nn.Dropout(0.5)
        self.dense_2 = nn.Linear(128, 64)
        self.dropout_2 = nn.Dropout(0.5)
        self.predictions = nn.Linear(64, output_num)
        self.activation = nn.Sigmoid()

    def forward(self, x):
        x, _ = self.lstm_1(x)
        x, _ = self.lstm_2(x)
        # x, _ = self.lstm_3(x)
        # x, _ = self.lstm_4(x)
        x = self.flatten(x)
        x = self.dense_1(x)
        # x = self.activation(x)
        x = self.dropout_1(x)
        x = self.dense_2(x)
        # x = self.activation(x)
        x = self.dropout_2(x)
        x = self.predictions(x)
        #x = torch.softmax(x, dim=1)
        return x
    
class Dst(Dataset):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __len__(self):
        return len(self.x)
    
    def __getitem__(self, i):
        return self.x[i], self.y[i]
    
    
def DatatoTorch(x, y, size, device):
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
            model.train()
            x, y = data
            optimizer.zero_grad()
            outputs = model(x)
            loss = criterion(outputs, y)
            loss.backward()
            loss_list.append(loss.item())
            optimizer.step()

            model.eval()  #dropout

            # print('Epoch: %d, Loss: %.3f' % (epoch, loss.item()))
        if(epoch%10==0):
            state = {'net':model.state_dict(), 'optimizer':optimizer.state_dict(), 'epoch':epoch}
            torch.save(state, name)

        print('Epoch: %d, Loss: %.3f' % (epoch, mean(loss_list)))
        if(math.isnan(mean(loss_list))):
            print("Something wrong!")
            break
        loss_list=[]
    pass


def result(model, x, y):
    list=[[]]
    print(x.shape)
    for i, data in enumerate(x):
        with torch.no_grad():
            outputs = model(torch.unsqueeze(x[i], dim=0)).cpu()
            print(i)
            list.append(outputs[0].numpy().tolist())
    data=pd.DataFrame(list)
    return data