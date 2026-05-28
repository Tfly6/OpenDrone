import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 设置字体参数
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42

# 读取数据
df = pd.read_csv("./rm_performance_log.csv")

index = df.columns[1:]  # 获取列名（去除第一列TIME_STAMPE）
f, ax = plt.subplots()
f.set_figheight(4)
f.set_figwidth(18)

all_data = []

for i in index:
    all_data.append(df[i])
    print(i, ":", np.average(df[i]) * 1000, " ms")

# 绘制箱型图，并隐藏异常值
plt.boxplot(all_data, showfliers=False)

# 设置横轴的标签
plt.xticks(ticks=np.arange(1, len(index) + 1), labels=index)

plt.ylabel("Time Consuming")
plt.xlabel("Data Labels")  # 可选：添加横轴名称
plt.show()
