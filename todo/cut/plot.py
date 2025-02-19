import pandas as pd
import matplotlib.pyplot as plt


path = 'Data/'

csv_file = path + "POSE6.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
pose = pd.DataFrame(csv_data)

# 绘制POSE6
plt.figure(figsize=(10, 6))
plt.plot(pose.iloc[:, 0], label='x')
plt.plot(pose.iloc[:, 1], label='y')
plt.plot(pose.iloc[:, 2], label='z')
plt.legend()
plt.show()


csv_file = path + "FORCE6.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
force = pd.DataFrame(csv_data)

# 绘制FORCE6
plt.figure(figsize=(10, 6))
plt.plot(force.iloc[:, 0], label='f1')
plt.plot(force.iloc[:, 1], label='f2')
plt.plot(force.iloc[:, 2], label='f3')
plt.legend()
plt.show()

csv_file = path + "K6.csv"
csv_data = pd.read_csv(csv_file)#防止弹出警告
K = pd.DataFrame(csv_data)

# 绘制FORCE6
plt.figure(figsize=(10, 6))
plt.plot(K.iloc[:, 1], label='k1')
plt.legend()
plt.show()
