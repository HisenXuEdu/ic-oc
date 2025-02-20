import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# # 读取数据
# pose = pd.read_csv('./demo/polish/data/pose.csv')
# force = pd.read_csv('./demo/polish/data/force.csv')

# # 取100到350的数据
# pose = pose[50:350]
# force = force[50:350]
# # 加入头部
# pose.columns = ['x', 'y', 'z']
# force.columns = ['x', 'y', 'z', 'rx', 'ry', 'rz']

# # 绘制数据
# # 设置时间序列，数据中每20个点1秒
# time = np.arange(0, (pose.shape[0])/15, 1/15)
# # 两张子图
# fig, ax = plt.subplots(2, 1, figsize=(8, 4))
# # 绘制位置
# ax[0].plot(time, pose['x']/1000, label=r'$x$', color='blue', linewidth=1.5)
# ax[0].plot(time, pose['y']/1000, label=r'$y$', color='green', linewidth=1.5)
# ax[0].plot(time, pose['z']/1000, label=r'$z$', color='red', linewidth=1.5)
# # 添加网格线
# ax[0].grid(color='gray', linestyle='--', linewidth=0.5)
# ax[0].set_xlim(0, 20)
# ax[1].set_xticks(np.arange(0, 20, 5))
# ax[0].set_ylim(-0.2, 0.2)
# ax[0].set_ylabel(r'Position(m)', fontsize=16)
# # 添加图例
# ax[0].legend(fontsize=14, loc='upper right')
# # 刻度字体大小
# ax[0].tick_params(labelsize=14)
# # 隐藏x轴刻度,但是绘制网格线
# ax[0].tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)

# # 绘制力
# ax[1].plot(time, force['x'], label=r'$fx$', color='blue', linewidth=1.5)
# ax[1].plot(time, force['y'], label=r'$fy$', color='green', linewidth=1.5)
# ax[1].plot(time, force['z'], label=r'$fz$', color='red', linewidth=1.5)
# # 添加网格线
# ax[1].grid(color='gray', linestyle='--', linewidth=0.5)
# ax[1].set_ylim(-10, 10)
# # 添加标题和坐标轴标签
# ax[1].set_xlabel(r'Time(s)', fontsize=16)
# ax[1].set_xlim(0, 20)
# ax[1].set_xticks(np.arange(0, 20, 5))
# ax[1].set_ylabel(r'Force(N)', fontsize=16, labelpad=8)
# # 添加图例
# ax[1].legend(fontsize=14, loc='upper right')
# # 刻度字体大小
# ax[1].tick_params(labelsize=14)

# plt.show()


# 读取数据
pose = pd.read_csv('./demo/polish/data/pose_e.csv')
force = pd.read_csv('./demo/polish/data/force_e.csv')

# 取100到350的数据
pose = pose[280:620]
force = force[280:620]
# 加入头部
pose.columns = ['x', 'y', 'z']
force.columns = ['x', 'y', 'z']

# 绘制数据
# 设置时间序列，数据中每20个点1秒
time = np.arange(0, (pose.shape[0])/15, 1/15)
# 两张子图
fig, ax = plt.subplots(2, 1, figsize=(10, 5))
# 绘制位置
ax[0].plot(time, pose['x'], label=r'$R$', color='blue', linewidth=1.5)
ax[0].plot(time, pose['y'], label=r'$P$', color='green', linewidth=1.5)
ax[0].plot(time, pose['z'], label=r'$Y$', color='red', linewidth=1.5)
# 添加网格线
ax[0].grid(color='gray', linestyle='--', linewidth=0.5)
ax[0].set_xlim(0, 22.5)
ax[0].set_ylim(-40, 40)
ax[0].set_ylabel(r'Euler(deg)', fontsize=16, labelpad=10)
# 添加图例
ax[0].legend(fontsize=14)
# 刻度字体大小
ax[0].tick_params(labelsize=14)
# 隐藏x轴刻度,但是绘制网格线
ax[0].tick_params(axis='x', which='both', bottom=False, top=False, labelbottom=False)

# 绘制力
ax[1].plot(time, force['x'], label=r'$fx$', color='blue', linewidth=1.5)
ax[1].plot(time, force['y'], label=r'$fy$', color='green', linewidth=1.5)
ax[1].plot(time, force['z'], label=r'$fz$', color='red', linewidth=1.5)
# 添加网格线
ax[1].grid(color='gray', linestyle='--', linewidth=0.5)
ax[1].set_xlim(0, 22.5)
# 设置x轴刻度间隔
ax[1].set_xticks(np.arange(0, 22.5, 5))
ax[1].set_ylim(-20, 20)
# 添加标题和坐标轴标签
ax[1].set_xlabel(r'Time(s)', fontsize=16)
# 设置偏移10
ax[1].set_ylabel(r'Force(N)', fontsize=16, labelpad=10)
# 添加图例
ax[1].legend(fontsize=14, loc='lower right')
# 刻度字体大小
ax[1].tick_params(labelsize=14)

plt.show()
