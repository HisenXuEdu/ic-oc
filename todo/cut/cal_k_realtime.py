import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

#5.1
# 将父级目录加入到import的path中
current_dir = os.path.dirname(__file__)
parent_dir = os.path.dirname(current_dir)
pparent_dir = os.path.dirname(parent_dir)
sys.path.append(pparent_dir)


from emg.emg import *

emg = Emg_S(channel=4, host='127.0.0.1')

k = []

for i in range(100):
    data_EMG = emg.get_single()
    kx = data_EMG * [838014.4280499433, 1507693.2629336102, 523971.4153734604, 77018.15021548994]
    # 对kx求和
    kx = np.sum(kx)
    ky = data_EMG * [572682.873130085, 237572.79217439564, 2131824.090954872, 2336218.0881788656]
    ky = np.sum(ky)
    kz = data_EMG * [372434.07012185, 1220167.8851617225, 46693.71400183649, 701693.5158285548]
    kz = np.sum(kz)

    # kx = data_EMG * [727270.3605538673, 1045607.6883612757, 1346800.8740848245, 1396994.8710918184]
    # # 对kx求和
    # kx = np.sum(kx)
    # ky = data_EMG * [2205298.0207280647, 3173161.92703226, 4859256.22845457, 5184119.015207673]
    # ky = np.sum(ky)
    # kz = data_EMG * [2122055.470371023, 3281181.3305397043, 3850759.351688576, 3845885.231315585]
    # kz = np.sum(kz)

    print(kx, ky, kz)

    k.append([kx, ky, kz])

# 绘制K
k = np.array(k)
plt.plot(k[:, 0], label='kx')
plt.plot(k[:, 1], label='ky')
plt.plot(k[:, 2], label='kz')
plt.legend()
plt.show()


