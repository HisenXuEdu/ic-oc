from model.LSTM import *
from emg.emg import *

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

def generate():
     # 生成六维的x， 每个维度生成一个长度为200的随机序列
    x = []
    for i in range(6):
        x.append(np.random.rand(20))
    x = np.array(x)
    return x
    
win_len = 20
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = MyModel(6, win_len,1)
model.to(device)
checkpoint = torch.load('model_path/LSTMi1o1.pth', map_location=torch.device('cpu'))
model.load_state_dict(checkpoint['net'])
x = generate()
change_para_emg()
