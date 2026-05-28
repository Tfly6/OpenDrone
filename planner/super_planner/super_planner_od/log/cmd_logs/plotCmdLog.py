import pandas as pd
import matplotlib.pyplot as plt
import os
import glob
import numpy as np

# 获取当前目录下最新的 CSV 文件
def get_latest_csv():
    # 匹配所有 CSV 文件
    csv_files = glob.glob("*.csv")
    if not csv_files:
        raise FileNotFoundError("No CSV files found in the current directory.")

    # 找到修改时间最新的文件
    latest_file = max(csv_files, key=os.path.getmtime)
    return latest_file

# 移除全为 0 的前几行
def remove_zero_rows(data):
    # 保留非全零行
    data = data.loc[~(data == 0).all(axis=1)]
    return data

# 计算范数并找到最大差值的位置
def find_max_diff_index(data_x, data_y, data_z):
    # 计算每一行的范数
    norms = np.sqrt(data_x**2 + data_y**2 + data_z**2)
    # 计算相邻范数的差值
    diff = np.abs(np.diff(norms))
    # 找到最大差值的索引
    max_diff_index = np.argmax(diff)
    return max_diff_index, norms

# 获取最新的 CSV 文件路径
latest_csv = get_latest_csv()
print(f"Using latest CSV file: {latest_csv}")

# 读取 CSV 文件
data = pd.read_csv(latest_csv)

# 去掉前几行全为 0 的行
data = remove_zero_rows(data)

# 提取所需数据
time = data["time"]

# 提取位置、速度、加速度、jerk 数据
posi_x, posi_y, posi_z = data["posi_x"], data["posi_y"], data["posi_z"]
vel_x, vel_y, vel_z = data["vel_x"], data["vel_y"], data["vel_z"]
acc_x, acc_y, acc_z = data["acc_x"], data["acc_y"], data["acc_z"]
jerk_x, jerk_y, jerk_z = data["jerk_x"], data["jerk_y"], data["jerk_z"]
yaw, yaw_rate = data["yaw"], data["yaw_rate"]
# 计算每种数据的范数并找到最大差值的位置
posi_max_diff_idx, posi_norms = find_max_diff_index(posi_x, posi_y, posi_z)
vel_max_diff_idx, vel_norms = find_max_diff_index(vel_x, vel_y, vel_z)
acc_max_diff_idx, acc_norms = find_max_diff_index(acc_x, acc_y, acc_z)
jerk_max_diff_idx, jerk_norms = find_max_diff_index(jerk_x, jerk_y, jerk_z)

# 找到 'backup' 列为 2 对应的 'time' 值
backup_times = data.loc[data['backup'] == 2, 'time']
# compute the ration of backup
print("Ratio of backup: ", len(backup_times) / len(data))

#
print("Average Vel. Norm: ", np.mean(vel_norms))

# 定义统一的绘图风格
plot_style = {
    "linestyle": "--",   # 点划线
    "linewidth": 1.5,    # 线宽
    "marker": "o",       # 圆点标记
    "markersize": 4      # 标记点大小
}



# 创建 3x2 多图
fig, axs = plt.subplots(3, 2, figsize=(12, 15), sharex=True)

# 绘制位置数据
axs[0, 0].plot(time, posi_norms, label="Position Norm", color="blue", **plot_style)
axs[0, 0].axvline(time[posi_max_diff_idx], color="red", linestyle="-", label="Max Norm Diff")
axs[0, 0].set_title("Position Norm")
axs[0, 0].set_ylabel("Norm")
axs[0, 0].legend()
axs[0, 0].grid(True)


# 绘制速度数据
axs[0, 1].plot(time, vel_norms, label="Velocity Norm", color="green", **plot_style)
axs[0, 1].axvline(time[vel_max_diff_idx], color="red", linestyle="-", label="Max Norm Diff")
axs[0, 1].set_title("Velocity Norm")
axs[0, 1].set_ylabel("Norm")
axs[0, 1].legend()
axs[0, 1].grid(True)

# 绘制加速度数据
axs[1, 0].plot(time, acc_norms, label="Acceleration Norm", color="orange", **plot_style)
axs[1, 0].axvline(time[acc_max_diff_idx], color="red", linestyle="-", label="Max Norm Diff")
axs[1, 0].set_title("Acceleration Norm")
axs[1, 0].set_ylabel("Norm")
axs[1, 0].legend()
axs[1, 0].grid(True)

# 绘制 jerk 数据
axs[1, 1].plot(time, jerk_norms, label="Jerk Norm", color="purple", **plot_style)
axs[1, 1].axvline(time[jerk_max_diff_idx], color="red", linestyle="-", label="Max Norm Diff")
axs[1, 1].set_title("Jerk Norm")
axs[1, 1].set_ylabel("Norm")
axs[1, 1].legend()
axs[1, 1].grid(True)


# 绘制范数差最大点的时间标记
axs[2, 0].text(0.1, 0.5, f"Position Max Diff Time: {time[posi_max_diff_idx]:.2f}s", fontsize=10)
axs[2, 0].text(0.1, 0.4, f"Velocity Max Diff Time: {time[vel_max_diff_idx]:.2f}s", fontsize=10)
axs[2, 0].text(0.1, 0.3, f"Acceleration Max Diff Time: {time[acc_max_diff_idx]:.2f}s", fontsize=10)
axs[2, 0].text(0.1, 0.2, f"Jerk Max Diff Time: {time[jerk_max_diff_idx]:.2f}s", fontsize=10)

axs[2, 0].axis('off')
axs[2, 1].axis('off')



# 调整布局
plt.tight_layout()

import pandas as pd
import matplotlib.pyplot as plt
import os
import glob
import numpy as np

# 获取当前目录下最新的 CSV 文件
def get_latest_csv():
    # 匹配所有 CSV 文件
    csv_files = glob.glob("*.csv")
    if not csv_files:
        raise FileNotFoundError("No CSV files found in the current directory.")

    # 找到修改时间最新的文件
    latest_file = max(csv_files, key=os.path.getmtime)
    return latest_file

# 移除全为 0 的前几行
def remove_zero_rows(data):
    # 保留非全零行
    data = data.loc[~(data == 0).all(axis=1)]
    return data

# 计算范数并找到最大差值的位置
def find_max_diff_index(data_x, data_y, data_z):
    # 计算每一行的范数
    norms = np.sqrt(data_x**2 + data_y**2 + data_z**2)
    # 计算相邻范数的差值
    diff = np.diff(norms)
    # 找到最大差值的索引
    max_diff_index = np.argmax(np.abs(diff))
    return max_diff_index, norms, diff

# 获取最新的 CSV 文件路径
latest_csv = get_latest_csv()
print(f"Using latest CSV file: {latest_csv}")

# 读取 CSV 文件
data = pd.read_csv(latest_csv)

# 去掉前几行全为 0 的行
data = remove_zero_rows(data)

# 提取所需数据
time = data["time"]
posi_x, posi_y, posi_z = data["posi_x"], data["posi_y"], data["posi_z"]

# 计算 position norm 和差分
posi_max_diff_idx, posi_norms, posi_diffs = find_max_diff_index(posi_x, posi_y, posi_z)

# 定义统一的绘图风格
plot_style = {
    "linestyle": "--",   # 点划线
    "linewidth": 1.5,    # 线宽
    "marker": "o",       # 圆点标记
    "markersize": 4      # 标记点大小
}

# 创建 2x1 子图布局
fig, axs = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

# 绘制 position norm 曲线
axs[0].plot(time, posi_norms, label="Position Norm", color="blue", **plot_style)
axs[0].axvline(time[posi_max_diff_idx], color="red", linestyle="-", label="Max Norm Diff")
axs[0].set_title("Position Norm")
axs[0].set_ylabel("Norm")
axs[0].legend()
axs[0].grid(True)

# 绘制 position norm 的差分曲线
axs[1].plot(time[1:], posi_diffs, label="Position Norm Difference", color="green", **plot_style)
axs[1].axvline(time[posi_max_diff_idx + 1], color="red", linestyle="-", label="Max Difference")
axs[1].set_title("Position Norm Difference")
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Norm Difference")
axs[1].legend()
axs[1].grid(True)

# 调整布局
plt.tight_layout()

# 定义统一的绘图风格
plot_style = {
    "linestyle": "--",   # 点划线
    "linewidth": 1.5,    # 线宽
    "marker": "o",       # 圆点标记
    "markersize": 4      # 标记点大小
}

# 创建 5x1 多图
fig, axs = plt.subplots(5, 1, figsize=(12, 15), sharex=True)

# 绘制位置数据
axs[0].plot(time, posi_x, label="Position X", color="red", **plot_style)
axs[0].plot(time, posi_y, label="Position Y", color="green", **plot_style)
axs[0].plot(time, posi_z, label="Position Z", color="blue", **plot_style)
axs[0].set_title("Position (X, Y, Z)")
axs[0].set_ylabel("Position")
axs[0].legend()
axs[0].grid(True)

# 绘制速度数据
axs[1].plot(time, vel_x, label="Velocity X", color="red", **plot_style)
axs[1].plot(time, vel_y, label="Velocity Y", color="green", **plot_style)
axs[1].plot(time, vel_z, label="Velocity Z", color="blue", **plot_style)
axs[1].set_title("Velocity (X, Y, Z)")
axs[1].set_ylabel("Velocity")
axs[1].legend()
axs[1].grid(True)

# 绘制加速度数据
axs[2].plot(time, acc_x, label="Acceleration X", color="red", **plot_style)
axs[2].plot(time, acc_y, label="Acceleration Y", color="green", **plot_style)
axs[2].plot(time, acc_z, label="Acceleration Z", color="blue", **plot_style)
axs[2].set_title("Acceleration (X, Y, Z)")
axs[2].set_ylabel("Acceleration")
axs[2].legend()
axs[2].grid(True)

# 绘制 jerk 数据
axs[3].plot(time, jerk_x, label="Jerk X", color="red", **plot_style)
axs[3].plot(time, jerk_y, label="Jerk Y", color="green", **plot_style)
axs[3].plot(time, jerk_z, label="Jerk Z", color="blue", **plot_style)
axs[3].set_title("Jerk (X, Y, Z)")
axs[3].set_ylabel("Jerk")
axs[3].legend()
axs[3].grid(True)

# 绘制 yaw 和 yaw_rate 数据
axs[4].plot(time, yaw, label="Yaw", color="purple", **plot_style)
axs[4].plot(time, yaw_rate, label="Yaw Rate", color="brown", **plot_style)
axs[4].set_title("Yaw and Yaw Rate")
axs[4].set_xlabel("Time (s)")
axs[4].set_ylabel("Angle / Rate")
axs[4].legend()
axs[4].grid(True)

for ax in axs:
    for t in backup_times:
        ax.axvline(x=t, color='orange', alpha=0.3)

# 调整布局
plt.tight_layout()
# 显示图形
plt.show()
