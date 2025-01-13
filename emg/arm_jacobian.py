import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import threading
import time
from sympy import symbols, Matrix, cos, sin, pi

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
    
        # 定义符号变量
        theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1 theta2 theta3 theta4 theta5 theta6 theta7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1 d2 d3 d4 d5 d6 d7')
        a1, a2, a3, a4, a5, a6, a7 = symbols('a1 a2 a3 a4 a5 a6 a7')
        alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7 = symbols('alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 alpha7')

        # DH 参数，替换实际的数值
        self.DH_params = [
            (theta1, d1, a1, alpha1),
            (theta2, d2, a2, alpha2),
            (theta3, d3, a3, alpha3),
            (theta4, d4, a4, alpha4),
            (theta5, d5, a5, alpha5),
            (theta6, d6, a6, alpha6),
            (theta7, d7, a7, alpha7),
        ]
    
        self.J = Matrix.zeros(6, 7)
            
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
    
    def dh_transform(self, theta, d, a, alpha):
        """
        计算每个关节的变换矩阵
        """
        return Matrix([
            [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

    
    def compute_jacobian(self):
        """
        计算雅可比矩阵
        """

        T = Matrix.eye(4)
        for param in self.DH_params:
            T = T * self.dh_transform(*param)

        # 提取末端执行器的位移和旋转部分
        position = T[:3, 3]
        rotation = T[:3, :3]
        
        # 计算雅可比矩阵
        J = Matrix.zeros(6, 7)
        z = Matrix([0, 0, 1])


        # 计算位置部分雅可比矩阵（3x7）
        p = Matrix([0, 0, 0])  # 基座位置
        for i in range(7):
            # 对于每个关节i，计算该关节对末端执行器的影响
            T_i = Matrix.eye(4)
            for j in range(i):
                T_i = T_i * self.dh_transform(*self.DH_params[j])
            
            # 计算位置雅可比矩阵
            Jp = (T_i[:3, 3] - p).cross(z)
            J[:3, i] = Jp
            p = T_i[:3, 3]
            
            # 计算角度雅可比矩阵（3x7）
            Jv = T_i[:3, 2]
            J[3:, i] = Jv
        
        self.J = J
        return J
    
    def set_parameters(self, param_values):
        """
        param_values: 一个包含 DH 参数值的字典
        """
        self.DH_params = [
            (param_values['theta1'], param_values['d1'], param_values['a1'], param_values['alpha1']),
            (param_values['theta2'], param_values['d2'], param_values['a2'], param_values['alpha2']),
            (param_values['theta3'], param_values['d3'], param_values['a3'], param_values['alpha3']),
            (param_values['theta4'], param_values['d4'], param_values['a4'], param_values['alpha4']),
            (param_values['theta5'], param_values['d5'], param_values['a5'], param_values['alpha5']),
            (param_values['theta6'], param_values['d6'], param_values['a6'], param_values['alpha6']),
            (param_values['theta7'], param_values['d7'], param_values['a7'], param_values['alpha7']),
        ]

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

    param_values = {
    'theta0': -pi/2, 'theta1': pi/2, 'theta2': 0, 'theta3': pi/2, 'theta4': 0, 'theta5': -pi/2, 'theta6': pi/2, 'theta7': 0,
    'd0': 0, 'd1': 0, 'd2': 0, 'd3': 0.3, 'd4': 0, 'd5': 0.27, 'd6': 0, 'd7': 0,
    'a0': 0, 'a1': 0, 'a2': 0, 'a3': 0, 'a4': 0, 'a5': 0, 'a6': 0, 'a7': 0.08,
    'alpha0': -pi/2, 'alpha1': pi/2, 'alpha2': -pi/2, 'alpha3': pi/2, 'alpha4': -pi/2, 'alpha5': pi/2, 'alpha6': -pi/2, 'alpha7': pi
    }
    arm.set_parameters(param_values)


    # 计算雅可比矩阵
    J_value = arm.compute_jacobian()
    print("雅可比矩阵：")
    print(J_value)
    
    joints_update_thread = threading.Thread(target=update_joints, args=(arm,))
    joints_update_thread.daemon = True
    joints_update_thread.start()

    # arm_plot_thread = threading.Thread(target=arm.plot_arm)
    # arm_plot_thread.daemon = True
    # arm_plot_thread.start()
    arm.plot_arm()

    # 等待线程结束
    joints_update_thread.join()
    # arm_plot_thread.join()

