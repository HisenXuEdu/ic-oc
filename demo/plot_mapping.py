import pandas as pd
import matplotlib.pyplot as plt

#读取data.csv
data = pd.read_csv('data1.csv')
data2 = pd.read_csv('data2.csv')

y1 = data.iloc[:,0]
y2 = data2.iloc[:,0]
plt.figure(figsize=(10, 6))
plt.plot(y1, label='v_mapping')
plt.plot(y2, label='v')
plt.legend()
plt.show()

y1 = data.iloc[35:250,0]
# 对y1进行积分
p1 = y1.cumsum()/80
a1 = y1.diff()

y2 = data2.iloc[40:200,0]
# 对y2进行积分
p2 = y2.cumsum()/80
a2 = y2.diff()
# 对x进行差分后的数据进行绘图
plt.figure(figsize=(10, 6))
plt.plot(p1, y1, label='v_mapping')
plt.plot(p1, a1, label='a_mapping')
plt.plot(p2, y2, label='v')
plt.plot(p2, a2, label='a')

plt.legend()
plt.xlim(0, 0.6)


plt.show()

