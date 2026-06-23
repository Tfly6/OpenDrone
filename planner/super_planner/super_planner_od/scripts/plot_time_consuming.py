import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np

csv = pd.read_csv('../log/time_consuming.csv', header=None)
data = csv.to_numpy()

types = data[0, :]
data = data[1:, :]
d = []
Types = []
X_axis = []
Y_axis = []
# data = data.astype(float)
# for i in range(0, data.shape[0]):
#     data[i,6] = np.sum(data[i,0:5])
print(data.shape)
for i in range(0, data.shape[1]):
    type = types[i]
    for j in range(0, data.shape[0]):
        if(float(data[j, i]) == 0):
            continue
        X_axis.append(j)
        Y_axis.append(float(data[j, i])*1000)
        Types.append(type)



d = {"Type": Types,
     "Time": X_axis,
     "Value": Y_axis}
df = pd.DataFrame(d)
print(df)
sns.set(font_scale = 1.2)
# f, axes = plt.subplots(1, 2)

sns.relplot(x="Time", y="Value",
            hue="Type",
            data=df,
            # palette=palette,
            kind = "line")
# plt.xlabel("Sample ID")
# plt.ylabel("Time (ms)")

# sns.boxplot(x="Type", y="Value",
#             # hue="Type",
#             data=df,
#             # palette=palette,
#             ax = axes[1])
# sns.displot(
#     x="Time", y="Value",
#     hue="Type",
#     data=df,
#     kind="kde", height=6,
#     multiple="fill", clip=(0, None),
#     palette="ch:rot=-.25,hue=1,light=.75",
# )

plt.show()
