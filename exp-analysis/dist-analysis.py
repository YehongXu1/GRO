import math
import numpy as np


def exp_iter_num_func(frac, reqno):
    exp_iter_num = 1
    j = reqno
    while True:
        j = math.floor(j * frac)
        if j < 1:
            break
        exp_iter_num += 1
    return exp_iter_num


path1 = '/Users/xyh/Desktop/traffic-assignment/exp-analysis/data/cong10-2.txt'
path2 = '/Users/xyh/Desktop/traffic-assignment/exp-analysis/data/base10.txt'

frac = 0.3
linenum = 0
base_cost = {}
base_time = {}

invalid1 = [4185, 4182, 2160, 4131, 3417, 2226, 6320, 3655, 3450, 4464, 2924, 1360, 1892, 3185, 1292, 3360, 2318, 3192, 2516, 3180, 1120, 2772, 3450, 3024, 2924, 1064, 3306]
invalid2 = [4185, 4182, 2160, 4131, 3417, 2226, 6320, 3655, 3450, 4464, 2924, 1360, 1892, 3360, 2318, 3192, 2223, 1024, 1323, 1440, 2516, 1376, 2925, 3180, 1638, 1120, 2772, 1392, 3450, 2610, 3024, 2924, 1750]
invalid3 = [4185, 4182, 2160, 4131, 3417, 2226, 6320, 3655, 3450, 4464, 2924, 1360, 1892, 3192, 2925, 3180, 1120, 3655, 3024]
invalid_req = set()
invalid_req.update(invalid1)
invalid_req.update(invalid2)
invalid_req.update(invalid3)


with open(path2, 'r') as r:
    reqno = -1
    for line in r:
        linenum += 1
        if linenum <= 2:
            continue
        if len(line.strip().split()) < 1:
            reqno = -1

        elif line.startswith('reqNo'):
            reqno = int(line.strip().split(': ')[1])
        elif line.startswith('random'):
            base_cost[reqno] = int(line.strip().split(': ')[1])
            # if int(line.strip().split(': ')[1]) < 0:
            #     invalid_req.append(reqno)
        elif line.startswith('baseline') and reqno not in invalid_req:
            base_time[reqno] = int(line.strip().split(': ')[1])

linenum = 0

max_iter_num = -1
with open(path1, 'r') as r:
    req_dict_iter = {}
    req_dict_t = {}

    reqno = -1
    exp_iter_num = -1
    iter_value = []
    for line in r:
        linenum += 1
        if linenum <= 2:
            continue
        if len(line.strip().split()) < 1:
            # empty
            reqno = -1
            iter_value = []
        elif line.startswith('reqNo'):
            reqno = int(line.strip().split(': ')[1])
            # exp_iter_num = exp_iter_num_func(frac, reqno)
            exp_iter_num = 21
        elif line.strip().startswith('time'):
            # if len(iter_value) == exp_iter_num:
            if reqno not in invalid_req:
                req_dict_t[reqno] = int(line.strip().split(' = ')[1]) / reqno
                req_dict_iter[reqno] = iter_value
                if len(iter_value) > max_iter_num:
                    max_iter_num = len(iter_value)
        # else:
        #     invalid_req.append(reqno)

        elif len(line.strip().split(',')) >= 3:
            cost = int(line.strip().split(',')[2]) / reqno
            iter_value.append(cost)

iter_sum = {}
for i in range(0, max_iter_num):
    iter_sum[i] = []

for k, v in req_dict_iter.items():
    while len(v) < max_iter_num:
        v.append(v[-1])
    for i in range(0, len(v)):
        iter_sum[i].append(v[i])

iter_perform = []
for i in range(0, len(iter_sum)):
    if len(iter_sum[i]) == 0:
        continue
    avg_c = np.sum(iter_sum[i])
    avg_c = round(avg_c)
    iter_perform.append(avg_c)

total_time = round(np.sum(list(req_dict_t.values())) / len(req_dict_iter.keys()), 2)

base_time_value = 0
base_cost_value = 0
for k in req_dict_iter.keys():
    base_time_value += base_time[k] / k
    base_cost_value += round(base_cost[k] / k)

base_time_value = round(base_time_value / len(req_dict_iter.keys()), 2)

if __name__ == '__main__':
    print(1)
