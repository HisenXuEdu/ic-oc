import pandas as pd
import matplotlib.pyplot as plt



# 自适应
path = 'Data/cut5_16/const/'

csv_file = path + "POSE1.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
pose = pd.DataFrame(csv_data)


start = 0
end = 300

# 绘制POSE6
fig = plt.figure(figsize=(10, 7))
# 设置字体大小
plt.rcParams.update({'font.size': 12})
plt.rcParams['lines.linewidth'] = 1.6
fig.add_subplot(411)
# plt.plot(pose.iloc[start:end, 0]-200, label='X', color = 'firebrick')
# plt.plot(pose.iloc[start:end, 1], label='Y', color = 'g')
# plt.plot(pose.iloc[start:end, 2], label='Z', color = 'b')
plt.plot(pose.iloc[start:end, 0]-200, label='X')
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
csv_file = path + "FORCE1.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
force = pd.DataFrame(csv_data)
force = force*3
fig.add_subplot(412)
plt.plot(force.iloc[start:end, 0], label='Fx')
plt.plot(force.iloc[start:end, 1], label='Fy')
plt.plot(force.iloc[start:end, 2], label='Fz')
plt.ylabel('Force(N)', fontsize=14, labelpad=10)
plt.yticks(range(-80, 20, 30))
plt.ylim(-90, 20)
plt.legend(loc='upper right')
plt.grid(True)
plt.xticks([])


start_work = 0
end_work = 300
# 对y做差分
y = pose.iloc[start_work:end_work, 1]/1000
dy = y.diff()
# 如果dy>0, 则为0, 否则为dy
dy = dy.where(dy < 0, 0)
# 计算做的功
W = dy * force.iloc[start_work:end_work, 1]
print(W.sum())

start = 0
end = 300

# # 绘制HK
# csv_file = path + "K1.csv"
# csv_data = pd.read_csv(csv_file)#防止弹出警告
# K = pd.DataFrame(csv_data)
# fig.add_subplot(413)

# plt.plot((K.iloc[start:end, 0]*7000000-50)/1000, label='Ky', color='#d62728')
# plt.ylabel('HK(KN/m)', fontsize=14, labelpad=18)
# plt.legend(loc='upper right')
# plt.yticks([i * 0.2 for i in range(1, 4)])
# plt.ylim(0, 0.7)
# plt.grid(True)
# plt.xticks([])

# # 绘制RK
# fig.add_subplot(414)
# HK = K.iloc[start:end, 0]*7000000-50
# RK = 1000 - HK/837*800

# x = range(0, len(RK), 1)
# x = [i/10 for i in x]

# plt.plot(x, RK/1000, label='Ky')

# plt.ylabel('RK(KN/m)', fontsize=14, labelpad=18)
# plt.xlabel('Time(s)', fontsize=14, labelpad=10)
# plt.legend(loc='upper right')
# plt.yticks([i * 0.2 for i in range(1, 6)])
# plt.ylim(0.35, 0.99)
# plt.grid(axis='y')
# plt.show()


# 阈值
path = 'Data/cut5_16/th/'

csv_file = path + "POSE1.csv"
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
csv_file = path + "FORCE1.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
force = pd.DataFrame(csv_data)
force = force*3
fig.add_subplot(412)
plt.plot(force.iloc[start:end, 0], label='Fx')
plt.plot(force.iloc[start:end, 1], label='Fy')
plt.plot(force.iloc[start:end, 2]+8, label='Fz')
plt.ylabel('Force(N)', fontsize=14, labelpad=10)
plt.yticks(range(-80, 20, 30))
plt.ylim(-90, 20)
plt.legend(loc='upper right')
plt.grid(True)
plt.xticks([])


start_work = 40
end_work = 150
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
csv_file = path + "K1.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
K = pd.DataFrame(csv_data)
fig.add_subplot(413)
plt.plot((K.iloc[start:end, 0]*7000000-50)/1000, label='Ky')
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
RK = HK.where(HK < 200, 200)
# RK大于200的值为1000
RK = RK.where(RK == 200, 1000)

# 计算RK中200的比例 除以 HK中大于100的比例
print(((RK == 200).sum()/len(RK))/((HK > 100).sum()/len(HK)))


x = range(0, len(RK), 1)
x = [i/10 for i in x]

# 绘制Hk和Rk
fig = plt.figure(figsize=(15, 3))
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


# # 恒定刚度
# path = 'Data/cut5_16/const/'

# csv_file = path + "POSE2.csv"
# csv_data = pd.read_csv(csv_file)#防止弹出警告
# pose = pd.DataFrame(csv_data)


# start = 0
# end = 300


# # 绘制POSE6
# fig = plt.figure(figsize=(10, 7))
# # 设置字体大小
# plt.rcParams.update({'font.size': 12})
# plt.rcParams['lines.linewidth'] = 1.6
# fig.add_subplot(411)
# # plt.plot(pose.iloc[start:end, 0]-200, label='X', color = 'firebrick')
# # plt.plot(pose.iloc[start:end, 1], label='Y', color = 'g')
# # plt.plot(pose.iloc[start:end, 2], label='Z', color = 'b')
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
# csv_file = path + "FORCE2.csv"
# csv_data = pd.read_csv(csv_file)#防止弹出警告
# force = pd.DataFrame(csv_data)
# force = force*3
# fig.add_subplot(412)
# plt.plot(force.iloc[start:end, 0], label='Fx')
# plt.plot(force.iloc[start:end, 1], label='Fy')
# plt.plot(force.iloc[start:end, 2], label='Fz')
# plt.ylabel('Force(N)', fontsize=14, labelpad=10)
# plt.yticks(range(-80, 20, 30))
# plt.ylim(-90, 20)
# plt.legend(loc='upper right')
# plt.grid(True)
# plt.xticks([])


# start_work = 0
# end_work = 300
# # 对y做差分
# y = pose.iloc[start_work:end_work, 1]/1000
# dy = y.diff()
# # 如果dy>0, 则为0, 否则为dy
# dy = dy.where(dy < 0, 0)
# # 计算做的功
# W = dy * force.iloc[start_work:end_work, 1]
# print(W.sum())

# plt.show()


# move
# path = 'Data/'

# csv_file = path + "POSE6.csv"
# csv_data = pd.read_csv(csv_file)#防止弹出警告
# pose = pd.DataFrame(csv_data)

# start = 150
# end = 330



# # 绘制POSE6
# fig = plt.figure(figsize=(10, 4))
# # 设置字体大小
# plt.rcParams.update({'font.size': 12})
# plt.rcParams['lines.linewidth'] = 1.6
# fig.add_subplot(211)
# # plt.plot(pose.iloc[start:end, 0]-200, label='X', color = 'firebrick')
# # plt.plot(pose.iloc[start:end, 1], label='Y', color = 'g')
# # plt.plot(pose.iloc[start:end, 2], label='Z', color = 'b')
# plt.plot(pose.iloc[start:end, 0]-75, label='X')
# plt.plot(pose.iloc[start:end, 1], label='Y')
# plt.plot(pose.iloc[start:end, 2], label='Z')
# plt.ylabel('Pose(mm)', fontsize=14, labelpad=6)
# # 设置y轴间隔
# plt.yticks(range(-100, 110, 50))
# plt.ylim(-110, 135)
# plt.legend(loc='upper right')
# plt.grid(True)
# plt.xticks([])

# # 计算


# # 绘制Force
# csv_file = path + "FORCE6.csv"
# csv_data = pd.read_csv(csv_file)#防止弹出警告
# force = pd.DataFrame(csv_data)
# force = force*3

# x = range(0, end-start, 1)
# x = [i/4.8 for i in x]

# fig.add_subplot(212)
# plt.plot(x, force.iloc[start:end, 0], label='Fx')
# plt.plot(x, force.iloc[start:end, 1], label='Fy')
# plt.plot(x, force.iloc[start:end, 2], label='Fz')
# plt.ylabel('Force(N)', fontsize=14, labelpad=14)
# plt.yticks(range(-80, 20, 30))
# plt.ylim(-90, 40)
# plt.legend(loc='upper right')
# plt.grid(axis='y')
# plt.xlabel('Time(s)', fontsize=14, labelpad=10)
# plt.tight_layout()
# plt.show()