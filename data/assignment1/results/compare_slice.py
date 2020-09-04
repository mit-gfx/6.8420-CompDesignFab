import numpy as np
import os
import sys
import argparse
from plyfile import PlyData, PlyElement

def getArgs(args=sys.argv[1:]):
    parser = argparse.ArgumentParser(description="Parses command.")
    parser.add_argument("-i", "--input", help="Input ply.")
    parser.add_argument("-r", "--reference", help="Reference ply.")
    parser.add_argument("-t", "--threshold", default=1e-4, help="Reference ply.")
    options = parser.parse_args(args)
    return options

# get input and reference files
options = getArgs(sys.argv[1:])

# load points
with open(options.input, 'rb') as f:
    input_ply = PlyData.read(f)
    input_points = np.asarray([list(ele) for ele in input_ply['vertex'].data])
    input_points_order = np.lexsort((input_points[:,0],input_points[:,1],input_points[:,2]))

with open(options.reference, 'rb') as f:
    ref_ply = PlyData.read(f)
    ref_points = np.asarray([list(ele) for ele in ref_ply['vertex'].data])
    ref_points_order = np.lexsort((ref_points[:,0],ref_points[:,1],ref_points[:,2]))

# first judge if the number of points matches
if len(input_points_order) != len(ref_points_order):
    print("Not equal")
else:
    for i in range(len(input_points_order)):
        if (np.linalg.norm(input_points[input_points_order[i]] - ref_points[ref_points_order[i]], 1) > options.threshold):
            print("Not equal")
            quit()
    print("Equal")
    

