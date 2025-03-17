from matplotlib import pyplot as plt
import pandas as pd

path = 'demo/polish/data224/'
force_file = 'force_2025-03-17-15-05-09.csv'
pose_file = 'pose_2025-03-17-15-05-09.csv'

# 绘制力的图像
fig = plt.figure()
ax = fig.add_subplot(211)
force_list = pd.read_csv(path + force_file).values
ax.plot(force_list[:,:3])

ax = fig.add_subplot(212)
pose_list = pd.read_csv(path + pose_file).values
ax.plot(pose_list)

plt.show()



