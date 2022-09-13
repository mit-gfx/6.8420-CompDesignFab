import numpy as np
import os
import sys
import argparse
from collections import deque

def getArgs(args=sys.argv[1:]):
    parser = argparse.ArgumentParser(description="Parses command.")
    parser.add_argument("-i", "--input", help="Input ply.")
    parser.add_argument("-r", "--reference", help="Reference ply.")
    parser.add_argument("-t", "--threshold", default=1e-4, help="Reference ply.")
    options = parser.parse_args(args)
    return options

# get input and reference files
options = getArgs(sys.argv[1:])


def ReadPly(path):
    with open(path, 'r') as f:
        lines = f.readlines()
        new_contour_flag = True
        # get number of points
        num_points = int(lines[2][15:])
        contours = []
        contours_start_points = []
        for i in range(7,len(lines)):
            if lines[i] == "\n":                
                contour = np.asarray(contour)
                contour_order = np.lexsort((contour[:,0],contour[:,1],contour[:,2]))
                fetch_order = deque(range(len(contour)))
                fetch_order.rotate(-contour_order[0])
                fetch_order = list(fetch_order)
                # append shifted contour
                contours.append(contour[fetch_order,:])
                contours_start_points.append(contour[fetch_order[0],:])
                new_contour_flag = True
            else:    
                if new_contour_flag:
                    contour = []
                    new_contour_flag = False
                contour.append([float(val) for val in lines[i].split()])
        # check input validity
        if not contours:
            print("Error - file '%s' is empty" % path)
            quit()
        contours_start_points = np.asarray(contours_start_points)
        # reorder contours
        contours_start_points_order = np.lexsort((contours_start_points[:,0],contours_start_points[:,1],contours_start_points[:,2]))
        contours_start_points = contours_start_points[contours_start_points_order,:]
        contours = [contours[i] for i in contours_start_points_order]
    return contours, contours_start_points, num_points

input_contours, input_contours_start_points, input_num_points = ReadPly(options.input)
ref_contours, ref_contours_start_points, ref_num_points = ReadPly(options.reference)

# compare contours
if input_num_points != ref_num_points or input_contours_start_points.shape[0] != ref_contours_start_points.shape[0]:
    print('Not equal')
else:
    for i in range(len(input_contours)):
        if input_contours[i].shape[0] != ref_contours[i].shape[0]:
            print("Not equal")
            quit()
        for j in range(input_contours[i].shape[0]):
            if j == 0:
                if (np.linalg.norm(input_contours[i][j] - ref_contours[i][j], 1) > options.threshold):
                    print("Not equal")
                    quit()
            else:
                # compare both clockwise and counter-clockwise
                if (np.linalg.norm(input_contours[i][j] - ref_contours[i][j], 1) > options.threshold) and \
                   (np.linalg.norm(input_contours[i][j] - ref_contours[i][input_contours[i].shape[0] - j], 1) > options.threshold):
                    print("Not equal")
                    quit()
    print("Equal")