import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
import threading
from time import sleep
import threading


class Arm:
    def __init__(self):
        self.K = np.array([0.3, 0.4, 0.5])
        self.fig = plt.figure(figsize=(6, 6))  # 两张图，上面是3D图，下面是2D图
        gs = GridSpec(2, 1, height_ratios=[2, 1]) 
        # 两张图，上面是3D图，下面是2D图
        self.ax1 = self.fig.add_subplot(gs[0], projection='3d')
        self.ax2 = self.fig.add_subplot(gs[1])

        self.angle = np.array([0, 0, 0])
        
        pass

    def update_data(self, K, angle):
        """
        更新关节位置
        :param joints: 关节位置列表
        """
        self.K = K
        # 获取陀螺仪三维角度
        self.angle = angle


    def plot_arm(self):
        ani = FuncAnimation(self.fig, self.update_fig, frames=100, interval=100)
        plt.show()
    
    def update_fig(self, frame):
        self.ax1.clear()  # 清除当前图形
        self.ax2.clear()

        # 地面网格
        # x = np.linspace(-0.5, 0.5, 10)
        # y = np.linspace(-0.5, 0.5, 10)
        # X, Y = np.meshgrid(x, y)
        # Z = np.zeros_like(X)
        # self.ax1.plot_surface(X, Y, Z, color='green', alpha=0.3, edgecolor='black')

        roll, pitch, yaw = np.radians(self.angle)  # Example RPY angles
        R = self.rpy_to_rotation_matrix(roll, pitch, yaw)

        self.plot_axes(np.eye(3), self.ax1, label='Original', length = 0.3, color=['r', 'g', 'b'])  # Plot original axes
        # self.plot_axes(R, self.ax1, label='Rotated')           # Plot rotated axes
        self.plot_ellipsoid(R, self.ax1, center=[0, 0, 0], radii=self.K)
        # self.ax1.legend()

        # # 绘制xyz轴和陀螺仪角度
        # self.ax1.quiver(0, 0, 0, 0.3, 0, 0, color='r')
        # self.ax1.quiver(0, 0, 0, 0, 0.3, 0, color='g')
        # self.ax1.quiver(0, 0, 0, 0, 0, 0.3, color='b')
        # self.ax1.quiver(0, 0, 0, *self.angle, color='black')

        # 设置坐标轴范围
        self.ax1.set_xlim([-0.5, 0.5])
        self.ax1.set_ylim([-0.5, 0.5])
        self.ax1.set_zlim([-0.5, 0.5])

        # 计算出self.K沿着各轴的分量
        projected_K = self.calculate_projected_axis_lengths(self.K, R)
        print(projected_K, R)
        self.ax2.bar(['X', 'Y', 'Z'], projected_K, color=['b', 'b', 'b'])
        self.ax2.set_ylim([0, 1])


    def rpy_to_rotation_matrix(self, roll, pitch, yaw):
        """Convert RPY angles to a rotation matrix."""
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
        
        R = Rz @ Ry @ Rx
        return R
    
    def plot_axes(self, R, ax, origin=[0, 0, 0], label='', length=1.0, color=['r', 'g', 'b'], labels = ['X', 'Y', 'Z']):
        """Plot the coordinate axes after applying rotation."""
        axes = np.identity(3)
        transformed_axes = R @ axes
        
        colors = color
        labels = ['X', 'Y', 'Z']
        
        for i in range(3):
            ax.quiver(*origin, *transformed_axes[:, i], color=colors[i], label=f'{label} {labels[i]}', length=length)
        # 在末端添加坐标轴标签
        ax.text(transformed_axes[0, 0]*length, transformed_axes[1, 0]*length, transformed_axes[2, 0]*length, f'{labels[0]}', color=colors[0])
        ax.text(transformed_axes[0, 1]*length, transformed_axes[1, 1]*length, transformed_axes[2, 1]*length, f'{labels[1]}', color=colors[1])
        ax.text(transformed_axes[0, 2]*length, transformed_axes[1, 2]*length, transformed_axes[2, 2]*length, f'{labels[2]}', color=colors[2])

    def plot_ellipsoid(self, rotation_matrix, ax, center, radii):
        # 绘制椭球表面
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = radii[0] * np.outer(np.cos(u), np.sin(v))
        y = radii[1] * np.outer(np.sin(u), np.sin(v))
        z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
        # Rotate and translate the ellipsoid
        for i in range(len(x)):
            for j in range(len(x)):
                [x[i, j], y[i, j], z[i, j]] = np.dot(rotation_matrix, [x[i, j], y[i, j], z[i, j]]) + center

        ax.plot_surface(x, y, z, color='pink', alpha=0.5)

        # 外圈黑色线包裹
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = radii[0] * np.outer(np.cos(u), np.sin(v))
        y = radii[1] * np.outer(np.sin(u), np.sin(v))
        z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
        # Rotate and translate the ellipsoid
        for i in range(len(x)):
            for j in range(len(x)):
                [x[i, j], y[i, j], z[i, j]] = np.dot(rotation_matrix, [x[i, j], y[i, j], z[i, j]]) + center
        # 虚线
        ax.plot_wireframe(x, y, z, color='black', alpha=0.8, linewidth=0.5, linestyle='--')
    
    def calculate_projected_axis_lengths(self, radii, rotation_matrix):
        """Calculate the projected lengths of the ellipsoid along the x, y, and z axes."""
        projected_radii = np.zeros(3)
        for i in range(3):
            projected_radii[i] = np.linalg.norm(np.dot(rotation_matrix[:, i], radii))
        return projected_radii
    
def update_data(arm):
    angle = arm.angle
    K = arm.K
    while True:
        K = np.array([0.3, 0.4, 0.5])
        angle = np.array([angle[0], angle[1]+10, angle[2]+10])
        arm.update_data(K, angle)
        sleep(1)
    pass


if __name__ == '__main__':
    arm = Arm()
    update_thread = threading.Thread(target=update_data, args=(arm,))
    update_thread.daemon = True
    update_thread.start()
    arm.plot_arm()