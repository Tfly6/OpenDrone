import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42

df = pd.read_csv("./time_consuming.csv")

index = df.columns[1:]
f, ax = plt.subplots()
f.set_figheight(4)
f.set_figwidth(18)

stampes = df["TIME_STAMPE"]
stampes = np.array(stampes)
all_data = []

for i in index:
    all_data.append(df[i])
    print(i, ":", np.average(df[i]) * 1000, " ms")

plt.boxplot(all_data, showfliers=False)

plt.xticks(ticks=np.arange(1, len(index) + 1), labels=index)

plt.ylabel("Time Consuming")
plt.xlabel("Data Labels")
plt.show()
