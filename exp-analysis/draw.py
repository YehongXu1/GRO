import matplotlib.pyplot as plt

y1 = [395798, 385506, 347850, 332832, 328616, 302895, 301610, 290472, 280846, 276675, 275903]
y2 = [395798, 372169, 347738, 346001, 350081, 350451, 346962, 349751, 373370, 330358, 382320, 347661, 361309, 353300, 376544, 343077, 385785, 327116, 389982, 339204, 371232]
y3 = [395798, 384668, 354787, 341948, 363248, 333960, 362532, 334405, 356099, 322320, 367349, 341817, 376128, 337386, 361131, 349654, 363338, 335837, 388131, 313948, 361138]


x = [i for i in range(0, len(y3))]

while len(y1) < len(x):
    y1.append(y1[-1])

while len(y2) < len(x):
    y2.append(y2[-1])

l1, = plt.plot(x, y1)
l2, = plt.plot(x, y2)
l3, = plt.plot(x, y3)
l4 = plt.axhline(y=352985, color='r')

plt.xlabel("iteration no.")
plt.ylabel("sum of avg. cost")
plt.title('Dist = 50km without fixing')
plt.legend(handles=[l1, l2, l3, l4], labels=['frac = 0.3', 'thres = 0.67', 'threshold = 0.99', 'baseline'], loc='best')
plt.savefig('/Users/xyh/Desktop/traffic-assignment/exp-analysis/plots/cong50.png')
plt.show()

if __name__ == '__main__':
    print(1)
