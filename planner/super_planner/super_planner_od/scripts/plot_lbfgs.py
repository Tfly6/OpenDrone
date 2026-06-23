import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("./lbfgs.txt")
print(data.shape)
start_id =0
plt.subplot(231)
plt.plot(data[start_id:,0])
plt.title("cost")

plt.subplot(232)
plt.plot(data[start_id:,1])
plt.title("t1")

plt.subplot(233)
plt.plot(data[start_id:,2])
plt.title("t2")

plt.subplot(234)
plt.plot(data[start_id:,3])
plt.title("gt1")

plt.subplot(235)
plt.plot(data[start_id:,4])
plt.title("gt2")

# plt.subplot(236)
# plt.plot(data[:,11])
# plt.title("gp")

plt.show()