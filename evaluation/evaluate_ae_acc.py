import h5py
import numpy
from tqdm import tqdm
import os
import argparse
import numpy as np
import sys
sys.path.append("..")
from cadlib.macro import *

parser = argparse.ArgumentParser()
parser.add_argument('--src', type=str, default=None, required=True)
args = parser.parse_args()

TOLERANCE = 3

result_dir = args.src
filenames = sorted(os.listdir(result_dir))

# overall accuracy
avg_cmd_acc = [] # ACC_cmd
avg_param_acc = [] # ACC_param

# accuracy w.r.t. each command type
each_cmd_cnt = np.zeros((len(ALL_COMMANDS),))
each_cmd_acc = np.zeros((len(ALL_COMMANDS),))

# accuracy w.r.t each parameter
args_mask = CMD_ARGS_MASK.astype(np.float)
N_ARGS = args_mask.shape[1]
each_param_cnt = np.zeros([*args_mask.shape])
each_param_acc = np.zeros([*args_mask.shape])

file_count = 0

for name in tqdm(filenames):

    file_count = file_count + 1
    if file_count < 5419:
        continue
    path = os.path.join(result_dir, name)
    with h5py.File(path, "r") as fp:
        out_vec = fp["out_vec"][:].astype(np.int)
        gt_vec = fp["gt_vec"][:].astype(np.int)

    out_cmd = out_vec[:, 0]
    gt_cmd = gt_vec[:, 0]

    out_param = out_vec[:, 1:]
    gt_param = gt_vec[:, 1:]

    cmd_acc = (out_cmd == gt_cmd).astype(np.int)
    param_acc = []
    for j in range(len(gt_cmd)):
        cmd = gt_cmd[j]
        each_cmd_cnt[cmd] += 1
        each_cmd_acc[cmd] += cmd_acc[j]
        if cmd in [SOL_IDX, EOS_IDX]:
            continue

        if out_cmd[j] == gt_cmd[j]: # NOTE: only account param acc for correct cmd
            tole_acc = (np.abs(out_param[j] - gt_param[j]) < TOLERANCE).astype(np.int)
            # filter param that do not need tolerance (i.e. requires strictly equal)

            # 这里需要添加其他指令的容忍设置

            if cmd == EXT_IDX:
                tole_acc[14:16] = (out_param[j] == gt_param[j]).astype(np.int)[14:16]
                tole_acc[18] = (out_param[j] == gt_param[j]).astype(np.int)[18]
            elif cmd == ARC_IDX:
                tole_acc[3] = (out_param[j] == gt_param[j]).astype(np.int)[3]
            elif cmd == POCKET_IDX:
                tole_acc[14:16] = (out_param[j] == gt_param[j]).astype(np.int)[14:16]
            elif cmd == REV_IDX:
                tole_acc[18] = (out_param[j] == gt_param[j]).astype(np.int)[18]
            elif cmd == SELECT_IDX:
                tole_acc[-4:] = (out_param[j] == gt_param[j]).astype(np.int)[-4:]
            elif cmd == HOLE_IDX:
                tole_acc[-5] = (out_param[j] == gt_param[j]).astype(np.int)[-5]


            valid_param_acc = tole_acc[args_mask[cmd].astype(np.bool)].tolist()
            param_acc.extend(valid_param_acc)

            each_param_cnt[cmd, np.arange(N_ARGS)] += 1
            each_param_acc[cmd, np.arange(N_ARGS)] += tole_acc

    # 直接在这里将自身的所有指令精度平均了，实际上可以分开计算，统计所有数据中各指令的精度
    if len(param_acc) > 0:
        param_acc = np.mean(param_acc)
    else:
        param_acc = 0
    avg_param_acc.append(param_acc)
    cmd_acc = np.mean(cmd_acc)
    avg_cmd_acc.append(cmd_acc)

save_path = result_dir + "_acc_stat.txt"
fp = open(save_path, "w")
# overall accuracy (averaged over all data)

avg_cmd_acc = np.mean(avg_cmd_acc)
print("avg command acc (ACC_cmd):", avg_cmd_acc, file=fp)

for i in range(len(avg_param_acc)):
    if numpy.isnan(avg_param_acc[i]) or numpy.isinf(avg_param_acc[i]):
        avg_param_acc[i] = 0

avg_param_acc = np.mean(avg_param_acc)
print("avg param acc (ACC_param):", avg_param_acc, file=fp)

# acc of each command type

each_cmd_acc = each_cmd_acc / (each_cmd_cnt + 1e-6)
print("each command count:", each_cmd_cnt, file=fp)
print("each command acc:", each_cmd_acc, file=fp)

# acc of each parameter type
each_param_acc = each_param_acc * args_mask
each_param_cnt = each_param_cnt * args_mask
each_param_acc = each_param_acc / (each_param_cnt + 1e-6)
for i in range(each_param_acc.shape[0]):
    print(ALL_COMMANDS[i] + " param acc:", each_param_acc[i][args_mask[i].astype(np.bool)], file=fp)

fp.close()

with open(save_path, "r") as fp:
    res = fp.readlines()
    for l in res:
        print(l, end='')
