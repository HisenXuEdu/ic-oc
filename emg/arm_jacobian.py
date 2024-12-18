import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import threading
import time

class Arm:
    def __init__(self):
        self.joints = np.array([
            [0, 0, 0],    # 基座
            [0, 0, 0.6],  # 第1关节
            [0.3, 0, 0.6], # 第2关节
            [0.5, 0.2, 0.4]  # 末端执行器
        ])
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
    
    def plot_sphere(self, center, radius=0.05, color='red'):
        """
        绘制一个球体表示关节
        :param ax: 3D 坐标轴
        :param center: 球心坐标 [x, y, z]
        :param radius: 球的半径
        :param color: 球的颜色
        """
        u = np.linspace(0, 2 * np.pi, 20)  # 球的水平角
        v = np.linspace(0, np.pi, 20)      # 球的垂直角
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        self.ax.plot_surface(x, y, z, color=color, alpha=0.6)
    
    def update_joints(self, joints):
        self.joints = joints
    
    def update(self, frame):
        """
        更新关节位置
        :param joints: 关节位置列表
        """
        self.ax.clear()  # 清除当前图形

        # 地面网格
        x = np.linspace(-0.5, 0.5, 10)
        y = np.linspace(-0.5, 0.5, 10)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)
        self.ax.plot_surface(X, Y, Z, color='green', alpha=0.3, edgecolor='black')

        # 绘制连杆
        for i in range(len(self.joints) - 1):
            x_values = [self.joints[i][0], self.joints[i+1][0]]
            y_values = [self.joints[i][1], self.joints[i+1][1]]
            z_values = [self.joints[i][2], self.joints[i+1][2]]
            self.ax.plot(x_values, y_values, z_values, 'r-o', linewidth=5)
        
        # 绘制关节 (球体)
        for joint in self.joints:
            self.plot_sphere(joint, radius=0.05, color='blue')

        # 坐标轴标签
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-0.5, 0.5])
        self.ax.set_ylim([-0.5, 0.5])
        self.ax.set_zlim([0, 0.8])
        self.ax.set_title('3D Arm Joint Visualization')

        # 绘制末端坐标系
        self.ax.quiver(self.joints[-1][0], self.joints[-1][1], self.joints[-1][2], 
                0.2, 0, 0, color='blue', label='X-Axis')  # X轴
        self.ax.quiver(self.joints[-1][0], self.joints[-1][1], self.joints[-1][2], 
                0, 0.2, 0, color='green', label='Y-Axis')  # Y轴
        self.ax.quiver(self.joints[-1][0], self.joints[-1][1], self.joints[-1][2], 
                0, 0, 0.2, color='purple', label='Z-Axis')  # Z轴

        self.ax.legend()
    
    def plot_arm(self):
        ani = FuncAnimation(self.fig, self.update, frames=100, interval=100)
        plt.show()


def update_joints(arm):
    # 每隔0.1秒更新一次关节位置
    i = 0
    while True:
        # 末端执行器画圈
        joints = np.array([
            [0, 0, 0],    # 基座
            [0, 0, 0.6],  # 第1关节
            [0.3, 0, 0.6], # 第2关节
            [0.5, 0.2*np.sin(i), 0.4]  # 末端执行器
        ])
        arm.update_joints(joints)
        i += 0.2
        # print(joints)
        time.sleep(0.3)

    
if __name__ == '__main__':
    arm = Arm()
    joints_thread = threading.Thread(target=update_joints, args=(arm,))
    joints_thread.daemon = True
    joints_thread.start()

    arm.plot_arm()