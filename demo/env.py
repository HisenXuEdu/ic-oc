import threading
from util.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
from ic.ic import IC
from util.plot import Plot
import copy

stop_thread = False

def plot_viz():
    global force, pose
    sleep(2)
    plt_pose=Plot(2000000,'POSE')
    plt_force=Plot(2000000,'FORCE')
    while True:
        plt_pose.plot(pose[2])
        plt_force.plot(force[2])
        if stop_thread:
            break

des_force = 20


class Env():
    def __init__(self):
        self.k = 5000

    def gen_f(self, x, y):
        ex = self.func_env_scope(y)
        if(x > ex):
            return 0
        return self.k * (ex - x)
    
    def func_env_sin(self, x):
        return 0.1 * np.sin(x)
    
    def func_env_scope(self, x):
        return 0.05 * x

    def func_env_random(self, x):
        return 0.15  * np.sin(x) * np.exp(-0.2*x)
    
    def func_env_vk(self, x):
        if x < 2:
            self.change_k(3000)
            return 0.05 * x
        elif x < 6:
            self.change_k(5000)
            return 0.08 * x - 0.06
        else:
            self.change_k(1000)
            return 0.03 * x + 0.24
    
    def func_flat(self, x):
        return 0.0
    
    def change_k(self, k):
        self.k = k


n = 1000
env = Env()
ic = IC(initial_pose=[0, 0, 0.2, 0, 0, 0], k=[128, 128, 0, 5, 5, 5], d=[32, 25, 150, 12, 12, 12], forward_force=np.array([0, 0, des_force, 0, 0, 0]), loop_rate=100)
ic_c = IC(initial_pose=[0, 0, 0.2, 0, 0, 0], k=[128, 128, 0, 5, 5, 5], d=[32, 25, 150, 12, 12, 12], forward_force=np.array([0, 0, des_force, 0, 0, 0]), loop_rate=100)
ic_raw = IC(initial_pose=[0, 0, 0.2, 0, 0, 0], k=[128, 128, 0, 5, 5, 5], d=[32, 25, 150, 12, 12, 12], forward_force=np.array([0, 0, des_force, 0, 0, 0]), loop_rate=100)
 
pose = [0, 0, 0.01]
pose_c = [0, 0, 0.01]
pose_raw = [0, 0, 0.01]
pose_list = np.empty((n, 3))
pose_list_c = np.empty((n, 3))
pose_list_raw = np.empty((n, 3))

force = np.zeros(6)
force_c = np.zeros(6)
force_raw = np.zeros(6)
force_list = np.empty((n, 6))
force_list_c = np.empty((n, 6))
force_list_raw = np.empty((n, 6))

record = threading.Thread(target=plot_viz)
record.daemon = True
record.start()

y = 0
for i in range(n):

    # 模拟0.02s的延迟
    
    

    y += 0.01
    force[2] = env.gen_f(pose[2], y)
    # force[2] = 0 if i < 3 else env.gen_f(pose_list[i-2][2], y)
    # 传输力的深拷贝
    pose, euler = ic.compute_admittance_env(copy.deepcopy(force))
    pose_list[i] = pose
    force_list[i] = force

    force_c[2] = env.gen_f(pose_c[2], y)
    # force_c[2] = 0 if i < 3 else env.gen_f(pose_list_c[i-2][2], y)
    pose_c, euler = ic_c.compute_admittance_ff(copy.deepcopy(force_c))
    pose_list_c[i] = pose_c
    force_list_c[i] = force_c

    force_raw[2] = env.gen_f(pose_raw[2], y)
    # force_raw[2] = 0 if i < 3 else env.gen_f(pose_list_raw[i-2][2], y)
    pose_raw, euler = ic_raw.compute_admittance_env_raw(copy.deepcopy(force_raw))
    pose_list_raw[i] = pose_raw
    force_list_raw[i] = force_raw

    # sleep(0.002)

# 结束线程
stop_thread = True

# 绘制图像
import matplotlib.pyplot as plt
plt.figure(figsize=(7, 3.5))
plt.plot(np.arange(n)/100, pose_list[:,2], label=r'$x_c$', color='blue', linewidth=1.5)
# plt.plot(np.arange(n)/100, pose_list_c[:,2], label=r'$x_c$', color='green', linewidth=1.5)
# 绘制环境曲线
plt.plot(np.arange(n)/100, [env.func_flat(0.01*i) for i in range(n)], label=r'$x_e$', color='r', linestyle='--', linewidth=1.5)
# 添加网格线
plt.grid(color='gray', linestyle='--', linewidth=0.5)
plt.ylim(-0.1, 0.4)

# 添加标题和坐标轴标签
plt.xlabel("Time(s)", fontsize=16)
plt.ylabel("Pose(m)", fontsize=18, labelpad=10)

# 添加图例
plt.legend(fontsize=13)
# 刻度字体大小
plt.xticks(fontsize=15)  # X 轴刻度
# 设置刻度间隔
# plt.yticks(np.arange(-0.1, 0.8, step=0.1))  # Y 轴刻度间隔为 0.1
plt.yticks(fontsize=15, )  # Y 轴刻度

# plt.ylim(-0.12, 0.32)

plt.tight_layout()

# plt.figure()

# plt.plot(np.arange(n)/100, pose_list_c[:,2], label=r'$x_c$', color='green', linewidth=1.5)




# # 绘制环境曲线
# fig = plt.figure(figsize=(7, 3.5))
# plt.plot(np.arange(200) / 100, [env.func_env_vk(0.01 * i) for i in range(200)],
#          label=r'$x_{e1}, k=3000$', color='#FF9999', linestyle='--', linewidth=1.5)  # 浅红色
# plt.plot(np.arange(200, 600) / 100, [env.func_env_vk(0.01 * (i + 200)) for i in range(400)],
#          label=r'$x_{e2}, k=5000$', color='#FF6666', linestyle='--', linewidth=1.5)  # 中红色
# plt.plot(np.arange(600, 1000) / 100, [env.func_env_vk(0.01 * (i + 600)) for i in range(400)],
#          label=r'$x_{e3}, k=1000$', color='#FF3333', linestyle='--', linewidth=1.5)  # 深红色

# plt.plot(np.arange(n) / 100, pose_list[:, 2], label=r'$x_c$', color='blue', linewidth=1.5)
# # plt.plot(np.arange(n)/100, pose_list_c[:,2], label=r'$x_c$', color='green', linewidth=1.5)

# # 添加网格线
# plt.grid(color='gray', linestyle='--', linewidth=0.5)
# # plt.ylim(-0.15, 0.15)

# # 添加标题和坐标轴标签
# # plt.title(r'Pose', fontsize=16)
# plt.xlabel("Time(s)", fontsize=16)
# plt.ylabel("Pose(m)", fontsize=18, labelpad=10)

# # 添加图例
# plt.legend(fontsize=13)
# # 刻度字体大小
# plt.xticks(fontsize=15)  # X 轴刻度
# plt.yticks(fontsize=15)  # Y 轴刻度

# plt.tight_layout()


plt.figure(figsize=(7, 3.5))
plt.plot(np.arange(n)/100, force_list[:,2], label=r'$F_e$', color='blue', linewidth=1.5)
# plt.plot(np.arange(n)/100, force_list_raw[:,2], label=r'$F_r$', color='orange', linewidth=1.5)
# plt.plot(np.arange(n)/100, force_list_c[:,2], label=r'$F_c$', color='green', linewidth=1.5)
# plt.ylim(0, 100)
# 绘制期望力虚线
plt.plot(np.arange(n)/100, np.ones(n) * des_force, label=r'$F_d$', color = 'r', linestyle='--', linewidth=1.5)
# 添加网格线
plt.grid(color='gray', linestyle='--', linewidth=0.5)

# 添加标题和坐标轴标签
# plt.title(r'Force', fontsize=18)
plt.xlabel("Time(s)", fontsize=16)
plt.ylabel("Force(N)", fontsize=16, labelpad=10)

plt.yticks(np.arange(-0, 50, step=10))
plt.ylim(-2, 25)

# 计算力峰值
print(np.max(force_list[:,2]))
print(np.max(force_list_raw[:,2]))

# 添加图例
plt.legend(fontsize=14)
# 刻度字体大小
plt.xticks(fontsize=15)  # X 轴刻度
plt.yticks(fontsize=15)  # Y 轴刻度

plt.tight_layout() 
plt.show()
