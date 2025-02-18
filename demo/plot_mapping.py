import pandas as pd
import matplotlib.pyplot as plt

#读取data.csv
data = pd.read_csv('data.csv')

# # 取第一列数据
# print(data.shape)
# x=data.iloc[:,0]
# # 对x进行差分
# diff = x.diff()

# # 对x进行差分后的数据进行绘图
# plt.figure(figsize=(10, 6))
# plt.plot(diff)
# plt.title('diff of x')

y = data.iloc[50:350,0]
a = y.diff()
# 对x进行差分后的数据进行绘图
plt.figure(figsize=(10, 6))
plt.plot(y)
plt.plot(a)
plt.title('diff of x')




plt.show()

