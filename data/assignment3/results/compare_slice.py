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
def ReadPly(path):
    with open(path, 'r') as f:
        lines = f.readlines()
        new_contour_flag = True
        # get number of points
        num_points = int(lines[2][15:])
        points = []
        for i in range(7,len(lines)):
            if lines[i] == "\n":                
                pass
            else:    
                points.append([float(val) for val in lines[i].split()])
        # check input validity
        if not points:
            print("Error - file '%s' is empty" % path)
            quit()
        # reorder contours
        points = np.asarray(points)
        if points.ndim != 2 or points.shape[1] != 3:
            print("Error - file '%s' has invalid content" % path)
            quit()
        points_order = np.lexsort((points[:,0],points[:,1],points[:,2]))
    return points, points_order, num_points

input_points, input_points_order, input_num_points = ReadPly(options.input)
ref_points, ref_points_order, ref_num_points = ReadPly(options.reference)


# first judge if the number of points matches
if input_num_points != ref_num_points:
    print("Not equal")
else:
    for i in range(len(input_points_order)):
        if (np.linalg.norm(input_points[input_points_order[i]] - ref_points[ref_points_order[i]], 1) > options.threshold):
            print("Not equal")
            quit()
    print("Equal")