import pandas as pd
import matplotlib.pyplot as plt



# 自适应
# path = 'Data/'

# csv_file = path + "POSE6.csv"
# csv_data = pd.read_csv(csv_file)#防止弹出警告
# pose = pd.DataFrame(csv_data)


# start = 300
# end = 490

# start = 280
# end = 430

# start = 0
# end = 1000

# start = 295
# end = 500

# # 绘制POSE6
# fig = plt.figure(figsize=(10, 8))
# # 设置字体大小
# plt.rcParams.update({'font.size': 12})
# fig.add_subplot(411)
# plt.plot(pose.iloc[start:end, 0]-200, label='X')
# plt.plot(pose.iloc[start:end, 1], label='Y')
# plt.plot(pose.iloc[start:end, 2], label='Z')
# plt.ylabel('Pose(mm)', fontsize=14, labelpad=10)
# # 设置y轴间隔
# plt.yticks(range(-90, 20, 30))
# plt.ylim(-100, 10)
# plt.legend(loc='upper right')
# plt.grid(True)
# plt.xticks([])

# # 计算


# # 绘制Force
# csv_file = path + "FORCE6.csv"
# csv_data = pd.read_csv(csv_file)#防止弹出警告
# force = pd.DataFrame(csv_data)
# fig.add_subplot(412)
# plt.plot(force.iloc[start:end, 0], label='Fx')
# plt.plot(force.iloc[start:end, 1], label='Fy')
# plt.plot(force.iloc[start:end, 2], label='Fz')
# plt.ylabel('Force(N)', fontsize=14, labelpad=10)
# plt.yticks(range(-20, 10, 10))
# plt.ylim(-30, 10)
# plt.legend(loc='upper right')
# plt.grid(True)
# plt.xticks([])


# start_work = 300
# end_work = 460
# # 对y做差分
# y = pose.iloc[start_work:end_work, 1]/1000
# dy = y.diff()
# # 如果dy>0, 则为0, 否则为dy
# dy = dy.where(dy < 0, 0)
# # 计算做的功
# W = dy * force.iloc[start_work:end_work, 1]
# print(W.sum())

# start = 580
# end = 1000

# # 绘制HK
# csv_file = path + "K6.csv"
# csv_data = pd.read_csv(csv_file)#防止弹出警告
# K = pd.DataFrame(csv_data)
# fig.add_subplot(413)
# plt.plot((K.iloc[start:end, 1]*7000000-50)/1000, label='Ky')
# plt.ylabel('HK(KN/m)', fontsize=14, labelpad=18)
# plt.legend(loc='upper right')
# # plt.yticks(range(0, 10, 10))
# plt.grid(True)
# plt.xticks([])

# # 绘制RK
# fig.add_subplot(414)
# HK = K.iloc[start:end, 1]*7000000-50
# RK = 1000 - HK/837*800

# x = range(0, len(RK), 1)
# x = [i/10 for i in x]

# plt.plot(x, RK/1000, color='brown', label='Ky')

# plt.ylabel('RK(KN/m)', fontsize=14, labelpad=18)
# plt.xlabel('Time(s)', fontsize=14, labelpad=10)
# plt.legend(loc='upper right')
# plt.grid(axis='y')
# plt.show()


# 阈值
path = 'Data/cut2_19/'

csv_file = path + "POSE6.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
pose = pd.DataFrame(csv_data)


start = 0
end = 210


# 绘制POSE6
fig = plt.figure(figsize=(10, 8))
# 设置字体大小
plt.rcParams.update({'font.size': 12})
fig.add_subplot(411)
plt.plot(pose.iloc[start:end, 0]-108, label='X')
plt.plot(pose.iloc[start:end, 1], label='Y')
plt.plot(pose.iloc[start:end, 2], label='Z')
plt.ylabel('Pose(mm)', fontsize=14, labelpad=10)
# 设置y轴间隔
plt.yticks(range(-90, 20, 30))
plt.ylim(-100, 10)
plt.legend(loc='upper right')
plt.grid(True)
plt.xticks([])

# 计算


# 绘制Force
csv_file = path + "FORCE6.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
force = pd.DataFrame(csv_data)
fig.add_subplot(412)
plt.plot(force.iloc[start:end, 0], label='Fx')
plt.plot(force.iloc[start:end, 1], label='Fy')
plt.plot(force.iloc[start:end, 2], label='Fz')
plt.ylabel('Force(N)', fontsize=14, labelpad=10)
plt.yticks(range(-20, 10, 10))
plt.ylim(-30, 10)
plt.legend(loc='upper right')
plt.grid(True)
plt.xticks([])


start_work = 300
end_work = 460
# 对y做差分
y = pose.iloc[start_work:end_work, 1]/1000
dy = y.diff()
# 如果dy>0, 则为0, 否则为dy
dy = dy.where(dy < 0, 0)
# 计算做的功
W = dy * force.iloc[start_work:end_work, 1]
print(W.sum())

start = 12
end = 410

# 绘制HK
csv_file = path + "K6.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
K = pd.DataFrame(csv_data)
fig.add_subplot(413)
plt.plot((K.iloc[start:end, 1]*7000000-50)/1000, label='Ky')
plt.ylabel('HK(KN/m)', fontsize=14, labelpad=18)
plt.legend(loc='upper right')
# plt.yticks(range(0, 10, 10))
plt.grid(True)
plt.xticks([])

# 绘制RK
fig.add_subplot(414)
HK = K.iloc[start:end, 1]*7000000-50
RK = 1000 - HK/837*800

x = range(0, len(RK), 1)
x = [i/10 for i in x]

plt.plot(x, RK/1000, color='brown', label='Ky')

plt.ylabel('RK(KN/m)', fontsize=14, labelpad=18)
plt.xlabel('Time(s)', fontsize=14, labelpad=10)
plt.legend(loc='upper right')
plt.grid(axis='y')


HK = K.iloc[start:end, 1]*7000000-50
# 如果HK>200, 则为200, 否则为1000
RK = HK.where(HK > 200, 200)
# RK大于200的值为1000
RK = RK.where(RK == 200, 1000)

# 计算RK中200的比例
print((RK == 200).sum()/len(RK))

x = range(0, len(RK), 1)
x = [i/10 for i in x]

# 绘制Hk和Rk
fig = plt.figure(figsize=(10, 3))
# 设置字体大小
plt.rcParams.update({'font.size': 12})
plt.plot(x,HK/1000, label='HK')
plt.plot(x,RK/1000, label='RK')
plt.ylabel('K(KN/m)', fontsize=14, labelpad=18)
plt.legend(loc='upper right')
plt.grid(axis='y')
# plt.yticks(range(0, 1.2, 0.3))
plt.ylim(-0.02, 1.1)
plt.xlabel('Time(s)', fontsize=14, labelpad=10)
plt.tight_layout()

plt.show()
