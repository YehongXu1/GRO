import matplotlib.pyplot as plt
import numpy as np

x = np.arange(3)

x1 = [0.15, 0.2, 0.19, 0.08]
x2 = [0.32, 0.35, 0.47, 0.22]
x3 = [0.48, 0.59, 0.71, 0.82]

ys = []
for i in range(0, len(x1)):
    ys.append([])
xs = [x1, x2, x3]
for i in range(0, 3):
    for j in range(0, len(xs[i])):
        ys[j].append(xs[i][j])

width = 0.2

# plot data in grouped manner of bar type

plt.bar(x - 0.3, ys[0], width)
plt.bar(x - 0.1, ys[1], width)
plt.bar(x + 0.1, ys[2], width)
plt.bar(x + 0.3, ys[3], width)

plt.xticks(x, ['10 km', '30 km', '50km'])
plt.xlabel("Distance")
plt.ylabel("Time (s)")
plt.legend(["frac = 0.3", "frac = 0.6", "frac = 0.9", 'baseline'])
plt.savefig('/Users/xyh/Desktop/traffic-assignment/exp-analysis/plots/unfix-time.png')
plt.show()

if __name__ == '__main__':
    print(1)