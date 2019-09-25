import os

testcases = ['bunny', 'fandisk', 'dragon', 'spot', \
             'bunny_fast', 'fandisk_fast', 'dragon_fast', 'spot_fast',\
             'bunny_with_hole', 'spot_with_hole']

for testcase in testcases:
    exist = True
    try:
        fp_res = open(os.path.join('results', testcase + '_voxel_info.txt'), 'r')
    except FileNotFoundError:
        exist = False
    if not exist:
        print('{:15s}: not found'.format(testcase))
        continue
    fp_std = open(os.path.join('std', testcase + '_voxel_info.txt'), 'r')

    data_res = fp_res.readline().split()
    nx_res, ny_res, nz_res = int(data_res[4]), int(data_res[5]), int(data_res[6])

    data_std = fp_std.readline().split()
    nx_std, ny_std, nz_std = int(data_std[4]), int(data_std[5]), int(data_std[6])

    if nx_res != nx_std or ny_res != ny_std or nz_res != nz_std:
        print("{:15s}: output dimensions mismatch".format(testcase))
        continue

    mismatch = 0
    for i in range((nx_std + 1) * (ny_std + 1)):
        s_res = fp_res.readline()
        s_std = fp_std.readline()
        for j in range(nz_std + 1):
            if s_res[j] != s_std[j]:
                mismatch += 1
    
    total = (nx_std + 1) * (ny_std + 1) * (nz_std + 1)
    similarity = float(total - mismatch) / float(total)

    print("{:15s}: solution and reference similarity = {:.3f}%".format(testcase, similarity * 100))
