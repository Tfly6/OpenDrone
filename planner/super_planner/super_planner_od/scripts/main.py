import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("./traj.txt")

jerk_norm = np.linalg.norm(data[:-2,10:13], axis=1, keepdims=False)
vel_norm = np.linalg.norm(data[:-2,4:7], axis=1, keepdims=False)
acc_norm = np.linalg.norm(data[:-2,7:10], axis=1, keepdims=False)
eval_t = data[:-2,0]
t1 = int(data[-1,0]*100)-1
t2 = int(data[-2,0]*100)-1

ts = (data[-1,0:1]*100) -1
ts = ts.astype(int)

pos_x = data[:-2,1]
pos_y = data[:-2,2]
plt.figure(figsize=(12,12))

plt.subplot(2,3,1)
tt = 0
plt.plot(pos_x,pos_y)
for t in ts:
    tt =tt + t
    print(tt)
    plt.plot(pos_x[tt], pos_y[tt],"o", color='coral')

# plt.plot(pos_x[t1],pos_y[t1],"o", color='coral')
plt.plot(pos_x[0], pos_y[0],"o", color='coral')
plt.axis("equal")
plt.xlabel("X/(m)")
plt.ylabel("Y/(m)")

plt.subplot(2,3,2)
plt.xlabel("t")
plt.ylabel("Vel Norm (m/s")
plt.plot(eval_t,vel_norm,"r")
tt = 0
for t in ts:
    tt =tt + t
    print(tt)
    plt.plot(eval_t[tt], vel_norm[tt],"o", color='coral')

plt.subplot(2,3,4)
plt.plot(eval_t,acc_norm,'b')
tt = 0
for t in ts:
    tt =tt + t
    print(tt)
    plt.plot(eval_t[tt], acc_norm[tt],"o", color='coral')
plt.xlabel("t")
plt.ylabel("Acc Norm (m/s")

plt.subplot(2,3,5)
plt.plot(eval_t,jerk_norm,'orange')
tt = 0
for t in ts:
    tt =tt + t
    print(tt)
    plt.plot(eval_t[tt], jerk_norm[tt],"o", color='coral')
plt.xlabel("t")
plt.ylabel("Jerk Norm (m/s")

plt.subplot(2,3,3)
plt.plot(data[:-2,9])
plt.plot(data[:-2,10])
plt.plot(data[:-2,11])

plt.subplot(2,3,6)
plt.plot(data[:-2,7])
plt.plot(data[:-2,8])
plt.plot(data[:-2,9])

plt.show()