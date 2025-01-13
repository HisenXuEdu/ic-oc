from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from math import radians, sin, cos
from matplotlib.animation import FuncAnimation
import threading
import time
from util.dobot_api import DobotApiDashboard, DobotApiMove
import re
from time import sleep
import socket



class RobotArm:
    def __init__(self, joint_num=6, joints_alpha=[90, 0, 0, 90, -90, 0], joints_a=[0, 0.425, 0.39225, 0, 0, 0], 
                 joints_d=[0.089159, 0, 0, 0.10915, 0.09465, 0.0823], joints_theta=[0, 0, 0, 0, 0, 0], 
                 joint_angle=[0, 0, 0, 0, 0, 0], modified_dh=False):
        self.joint_num = joint_num

        # --- Robotic Arm construction ---
        # DH参数表，分别用一个列表来表示每个关节的东西。
        # self.joints_alpha = [0, 90, 90, 90, 90, 90, 90]
        # self.joints_a = [0, 0, 0, 0, 0, 0, 0]
        # self.joints_d = [0.31, 0.0, 0.4, 0.0, 0.4, 0.0, 0.175]
        # self.joints_theta = [0, 180, 180, 180, 180, 180, 180]

        self.joints_alpha = joints_alpha
        self.joints_a = joints_a
        self.joints_d = joints_d
        self.joints_theta = joints_theta

        self.joint_angle = joint_angle

        self.modified_dh = modified_dh

        self.X = [0, 0, 0, 0, 0, 0, 0]
        self.Y = [0, 0, 0, 0, 0, 0, 0]
        self.Z = [0, 0, 0, 0, 0, 0, 0]
        self.T = []
        self.jacobian = np.zeros((6, self.joint_num))

        self.fig = plt.figure()
        # 两张图，上面是3D图，下面是2D图
        self.ax = self.fig.add_subplot(111, projection='3d')


        # 肌肉激活相关
        self.c1 = 0.1
        self.c2 = 0.1
        self.PB = 0.1
        self.PT = 0.1

    def set_axes_equal(self, ax):
    # 这一段是copy别人的。用处不是很大。
        '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
        cubes as cubes, etc..  This is one possible solution to Matplotlib's
        ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
        Input
        ax: a matplotlib axis, e.g., as output from plt.gca().
        '''

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    def dh_matrix(self, alpha, a, d, theta):
        """
        生成经典 DH 参数化的变换矩阵。
        
        参数:
        theta -- 关节角度（绕 Z 轴的旋转角度）
        d -- 关节偏移（沿 Z 轴的平移）
        a -- 连杆长度（沿 X 轴的平移）
        alpha -- 连杆扭转角度（绕 X 轴的旋转角度）
        
        返回:
        4x4 的变换矩阵
        """
        alpha = alpha / 180 * np.pi
        theta = theta / 180 * np.pi
        matrix = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
            ])
        return matrix

    def dh_matrix_modified(self, theta, d, a):
        """
        生成改进版 DH 参数化的变换矩阵。
        
        参数:
        theta -- 关节角度（绕 Z 轴的旋转角度）
        d -- 关节偏移（沿 Z 轴的平移）
        a -- 连杆长度（沿 X 轴的平移）
        
        返回:
        4x4 的变换矩阵
        """
        theta = theta / 180 * np.pi
        matrix = np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta), np.cos(theta), 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]
        ])
        return matrix
    
    def update_joints(self, joints):
        """
        更新关节位置
        :param joints: 关节位置列表
        """
        self.joint_angle = joints
        print("RobotArm:",self.joint_angle)
    
    def cal(self):
        #    DH参数转转换矩阵T---------------------
        joint_hm = []
        for i in range(self.joint_num):
            if self.modified_dh:
                joint_hm.append(self.dh_matrix_modified(self.joints_theta[i]+self.joint_angle[i], self.joints_d[i], self.joints_a[i]))
            else:    
                joint_hm.append(self.dh_matrix(self.joints_alpha[i], self.joints_a[i], self.joints_d[i], self.joints_theta[i]+self.joint_angle[i]))

        # -----------连乘计算----------------------
        for i in range(self.joint_num-1):
            joint_hm[i+1] = np.dot(joint_hm[i], joint_hm[i+1])    
        # Prepare the coordinates for plotting
        # for i in range(self.joint_num):
        #     print(np.round(joint_hm[i][:3, 3], 5))
        # 获取坐标值
        self.X = [hm[0, 3] for hm in joint_hm]
        self.Y = [hm[1, 3] for hm in joint_hm]
        self.Z = [hm[2, 3] for hm in joint_hm]
        self.T = joint_hm

        self.jacobian = np.zeros((6, self.joint_num))
        # 计算雅可比矩阵
        for i in range(self.joint_num):
            # 计算每个关节对末端执行器的影响
            z = joint_hm[i][:3, 2]
            o_n = joint_hm[-1][:3, 3]
            o_i = joint_hm[i][:3, 3]
            self.jacobian[:3, i] = np.cross(z, o_n - o_i)
            self.jacobian[3:, i] = z

    def cal_K(self):
        # 计算雅可比矩阵的伪逆
        J = self.jacobian
        J_inv = np.linalg.pinv(J)
        J_inv_T = J_inv.T
        acc = self.cal_acc()
        K_J = np.zeros((7,))
        G_j = np.zeros((7,))

        K = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        K_C = J_inv_T @ (acc*K_J - G_j) @ J_inv
        return K
    
    def cal_acc(self):
        exp_term = np.exp(-self.c2 * (self.PB + self.PT))
        numerator = self.c1 * (1 - exp_term)
        denominator = 1 + exp_term
        return 1 + numerator / denominator

    def plot_arm(self):
        ani = FuncAnimation(self.fig, self.update, frames=100, interval=100)
        plt.show()
    
    def update(self, frame):
        """
        更新关节位置
        :param joints: 关节位置列表
        """
        self.cal()
        self.ax.clear()  # 清除当前图形

        # 地面网格
        x = np.linspace(-0.5, 0.5, 10)
        y = np.linspace(-0.5, 0.5, 10)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)
        self.ax.plot_surface(X, Y, Z, color='green', alpha=0.3, edgecolor='black')

        # 绘制连杆
        for i in range(len(self.X) - 1):
            # x_values = [self.joints[i][0], self.joints[i+1][0]]
            # y_values = [self.joints[i][1], self.joints[i+1][1]]
            # z_values = [self.joints[i][2], self.joints[i+1][2]]
            x_values = [self.X[i], self.X[i+1]]
            y_values = [self.Y[i], self.Y[i+1]]
            z_values = [self.Z[i], self.Z[i+1]]
            self.ax.plot(x_values, y_values, z_values, 'r-o', linewidth=5)
        
        # 绘制关节 (球体)
        for joint in zip(self.X, self.Y, self.Z):
            self.plot_sphere(joint, radius=0.05, color='red')

        # 坐标轴标签
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-0.5, 0.5])
        self.ax.set_ylim([-0.5, 0.5])
        self.ax.set_zlim([0, 0.8])
        self.ax.set_title('3D Arm Joint Visualization')

        # 绘制末端执行器的坐标系
        self.plot_end_effector_axes()

    
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

    def plot_end_effector_axes(self):
        """
        绘制末端执行器坐标系
        """
        position = self.T[-1][:3, 3]  # 末端执行器的位置
        orientation = self.T[-1][:3, :3]  # 末端执行器的旋转矩阵
        # 绘制
        # 绘制原点
        self.ax.quiver(position[0], position[1], position[2], 0.1, 0, 0, color='r', length=0.1)
        self.ax.quiver(position[0], position[1], position[2], 0, 0.1, 0, color='g', length=0.1)
        self.ax.quiver(position[0], position[1], position[2], 0, 0, 0.1, color='b', length=0.1)

        # 绘制末端执行器的坐标系（X, Y, Z轴）
        self.ax.quiver(position[0], position[1], position[2], orientation[0, 0], orientation[1, 0], orientation[2, 0], color='r', length=0.1)
        self.ax.quiver(position[0], position[1], position[2], orientation[0, 1], orientation[1, 1], orientation[2, 1], color='g', length=0.1)
        self.ax.quiver(position[0], position[1], position[2], orientation[0, 2], orientation[1, 2], orientation[2, 2], color='b', length=0.1)


def update_joints(arm):
    # 每隔0.1秒更新一次关节位置
    joints = arm.joint_angle
    while True:
        # 最后一个关节的增加
        joints = [joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]+10, joints[7]]
        # 所有关节大于-360度小于360度
        joints = [j if j >= -360 else -360 for j in joints]
        joints = [j if j <= 360 else 360 for j in joints]
        arm.update_joints(joints)
        time.sleep(0.3)

def update_joints_cr5(arm, dashboard):
    # 每隔0.1秒更新一次关节位置
    joints = arm.joints_theta
    while True:
        # 最后一个关节的增加
        angle = dashboard.GetAngle()
        # 使用正则表达式提取花括号中的所有数字
        angle = re.search(r'\{([^\}]+)\}', angle).group(1)

        # 将提取出的字符串按逗号分隔并转换为浮动类型
        angle = list(map(float, angle.split(',')))

        print(angle)
        joints = angle
        # 所有关节大于-360度小于360度
        joints = [j if j >= -360 else -360 for j in joints]
        joints = [j if j <= 360 else 360 for j in joints]
        arm.update_joints(joints)
        time.sleep(0.3)

def update_joints_com(arm, port=65432):
    """通过本地电脑socket直接传输数据
    Args:
        arm (_type_): _description_
        port (int, optional): _description_. Defaults to 65432.
    """    
    # 创建一个TCP/IP套接字
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 连接到服务器
    client_socket.connect(('localhost', port))
    while True:
        # 接收数据
        data = client_socket.recv(1024)
        if not data:
            break
        angle = re.search(r'\{([^\}]+)\}', data.decode()).group(1)
        joints = list(map(float, angle.split(',')))

        # 所有关节大于-360度小于360度
        joints = [j if j >= -360 else -360 for j in joints]
        joints = [j if j <= 360 else 360 for j in joints]
        arm.update_joints(joints)


def pub_angle(dashboard):
    """发布关节角度
       如果需要展示关节变化，在代码开启新线程执行代码，并启动多进程执行robot程序
    Args:
        dashboard (_type_): cr5的dashboard对象
    """    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 绑定地址和端口
    server_address = ('localhost', 65432)
    server_socket.bind(server_address)

    # 监听连接请求
    server_socket.listen(1)

    print("等待客户端连接...")
    connection, client_address = server_socket.accept()

    try:
        print(f"连接成功: {client_address}")
        while True:
            # 最后一个关节的增加
            angle = dashboard.GetAngle()
            connection.sendall(angle.encode())
            sleep(0.2)

    finally:
        connection.close()
        server_socket.close()


if __name__ == '__main__':
    # 直接执行代码
    # robot_cr5 = RobotArm(joint_num=6, joints_alpha=[90, 0, 0, 90, -90, 0], joints_a=[0, 0.427, 0.357, 0, 0, 0], 
    #              joints_d=[0.147, 0, 0, 0.116, 0.116, 0.105], joints_theta=[0, 90, 0, 90, 0, 0], joint_angle=[0, 0, 0, 0, 0, 0], modified_dh=False)
    # joints_update_thread = threading.Thread(target=update_joints, args=(robot_cr5,))
    # joints_update_thread.daemon = True
    # joints_update_thread.start()
    # robot_cr5.plot_arm()

    # 接受本地关节角
    # 这里的第一个关机貌似没什么用，我看和第三个关节的效果一样
    arm = RobotArm(joint_num=7, joints_alpha=[-90, 90, -90, 90, -90, 90, -90, 180], joints_a=[0, 0, 0, 0, 0, 0, 0, 0.08], 
                 joints_d=[0, 0, 0, 0, 0.3, 0, 0.27, 0], joints_theta=[-90, 90, 0, 90, 0, -90, 90, 0],joint_angle=[0, 0, 0, 0, 0, 90, 0, 0], modified_dh=False)
    # joints_update_thread = threading.Thread(target=update_joints, args=(arm,))
    # joints_update_thread.daemon = True
    # joints_update_thread.start()
    arm.plot_arm()
