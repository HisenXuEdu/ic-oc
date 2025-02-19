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
p1 = (y1.cumsum()-0.38)/80
a1 = y1.diff()

y2 = data2.iloc[40:200,0]
# 对y2进行积分
p2 = (y2.cumsum()-0.25)/80
a2 = y2.diff()
# 对x进行差分后的数据进行绘图
# 创建一个绘图对象
fig, ax1 = plt.subplots(figsize=(12, 6))

# 绘制第一个曲线，使用 ax1
# ax1.plot(p2, y2, 'lightsteelblue', label='v')
ax1.plot(p2, y2, 'b', label='v', alpha=0.3)
ax1.plot(p1, y1, 'b-', label='v_mapping')
ax1.set_xlabel('Position(m)', fontsize=16, labelpad=5)
ax1.set_ylabel('Velocity(m/s)', color='b', fontsize=16, labelpad=10)
ax1.tick_params(axis='y', labelcolor='b', labelsize=13)
ax1.tick_params(axis='x', labelsize=13)
plt.legend(fontsize=14, loc='lower left')
fig.tight_layout()

# 创建第二个 y 轴，分享同一个 x 轴
ax2 = ax1.twinx()
# 绘制第二个曲线，使用 ax2
# ax2.plot(p2, a2, 'mistyrose', label='a', linestyle=':')
# 设置透明度
ax2.plot(p2, a2, 'r', label='a', linestyle='-', alpha=0.3)
ax2.plot(p1, a1, 'r-', label='a_mapping')
ax2.set_ylabel('Acceleration(m/s^2)', color='r', fontsize=16, labelpad=10)
ax2.tick_params(axis='y', labelcolor='r', labelsize=13)

plt.grid()
plt.legend(fontsize=14, loc='lower right')
plt.xlim(0, 0.6)
plt.ylim(-0.6, 0.1)
fig.tight_layout()


plt.show()



# import pandas as pd
# import matplotlib.pyplot as plt

# #读取data.csv
# data = pd.read_csv('data1.csv')
# data2 = pd.read_csv('data2.csv')

# y1 = data.iloc[:,0]
# y2 = data2.iloc[:,0]
# plt.figure(figsize=(10, 6))
# plt.plot(y1, label='v_mapping')
# plt.plot(y2, label='v')
# plt.legend()
# plt.show()

# y1 = data.iloc[35:250,0]
# # 对y1进行积分
# p1 = (y1.cumsum()-0.35)/80
# a1 = y1.diff()

# y2 = data2.iloc[40:200,0]
# # 对y2进行积分
# p2 = (y2.cumsum()-0.25)/80
# a2 = y2.diff()
# # 对x进行差分后的数据进行绘图
# # 创建一个绘图对象
# # 两张子图
# fig = plt.figure(figsize=(12, 6))
# ax1 = fig.add_subplot(212)
# # 绘制第一个曲线，使用 ax1
# # ax1.plot(p2, y2, 'lightsteelblue', label='v')
# ax1.plot(p2, y2, 'b', label='v', alpha=0.3)
# ax1.plot(p1, y1, 'b-', label='v_mapping')
# ax1.set_xlabel('Position(m)', fontsize=16, labelpad=5)
# ax1.set_ylabel('Velocity(m/s)', color='b', fontsize=16, labelpad=10)
# ax1.tick_params(axis='y', labelcolor='b', labelsize=13)
# ax1.tick_params(axis='x', labelsize=13)
# plt.legend(fontsize=14, loc='lower left')
# fig.tight_layout()

# # 创建第二个 y 轴，分享同一个 x 轴
# ax2 = fig.add_subplot(211)
# # 绘制第二个曲线，使用 ax2
# # ax2.plot(p2, a2, 'mistyrose', label='a', linestyle=':')
# # 设置透明度
# ax2.plot(p2, a2, 'r', label='a', linestyle='-', alpha=0.3)
# ax2.plot(p1, a1, 'r-', label='a_mapping')
# ax2.set_ylabel('Acceleration(m/s^2)', color='r', fontsize=16, labelpad=10)
# ax2.tick_params(axis='y', labelcolor='r', labelsize=13)

# plt.grid()
# plt.legend(fontsize=14, loc='lower right')
# plt.xlim(0, 0.6)
# plt.ylim(-0.6, 0.1)
# fig.tight_layout()


# plt.show()
