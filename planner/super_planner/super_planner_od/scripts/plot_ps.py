import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("./lbfgs.txt")
print(data.shape)
# 0     1   2-5 5-8 8-11    11-14 14-17
# cost  t   p1  p2  p3      p4    p5

start_id = 30
colors_i = ["r","g","b","y", "c"]
colors = ["ro","go","bo","yo","co"]

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
for i in range(0,1):
    print(i)
    pos_x = data[0:, 2+(i)*6]
    pos_y = data[0:, 3+(i)*6]
    pos_z = data[0:, 4+(i)*6]
    ax.plot(pos_x, pos_y,pos_z, colors_i[i])
    ax.plot(pos_x[0], pos_y[0],pos_z[0], colors[i])


plt.figure(2)
for i in range(0,1):
    print(i)
    pos_x = data[start_id:, 5+(i)*6]
    pos_y = data[start_id:, 6+(i)*6]
    # pos_z = data[:, 4+(i-1)*3]
    plt.plot(pos_x, pos_y, colors_i[i])
    plt.plot(pos_x[0], pos_y[0], colors[i])


plt.figure(3)
plt.plot(data[start_id:,0])
plt.figure(4)
plt.plot(data[start_id:,1])


plt.show()