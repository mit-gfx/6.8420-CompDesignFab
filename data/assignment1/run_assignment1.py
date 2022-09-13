import os
import sys
import time

def run(command):
    t0 = time.time()
    os.system(command)
    t1 = time.time()
    print('Command: %s finished in %f seconds...' % (command, t1 - t0))

# Assignment 1.1: basic voxelization.
run('./assignment1/run1 bunny.stl 2.0')
run('mv bunny_voxel.stl ../data/assignment1/')
run('./assignment1/run1 fandisk.stl 0.05')
run('mv fandisk_voxel.stl ../data/assignment1/')
run('./assignment1/run1 spot.stl 0.125')
run('mv spot_voxel.stl ../data/assignment1/')
run('./assignment1/run1 dragon.stl 0.05')
run('mv dragon_voxel.stl ../data/assignment1/')

# Assignment 1.2: fast voxelization.
run('./assignment1/run1 bunny.stl 2.0 fast')
run('mv bunny_voxel.stl ../data/assignment1/bunny_voxel_fast.stl')
run('mv bunny_voxel_info.txt ../data/assignment1/assignment1/')
run('./assignment1/run1 fandisk.stl 0.05 fast')
run('mv fandisk_voxel.stl ../data/assignment1/fandisk_voxel_fast.stl')
run('mv fandisk_voxel_info.txt ../data/assignment1/')
run('./assignment1/run1 spot.stl 0.125 fast')
run('mv spot_voxel.stl ../data/assignment1/spot_voxel_fast.stl')
run('mv spot_voxel_info.txt ../data/assignment1/')
run('./assignment1/run1 dragon.stl 0.05 fast')
run('mv dragon_voxel.stl ../data/assignment1/dragon_voxel_fast.stl')
run('mv dragon_voxel_info.txt ../data/assignment1/')

# Assignment 1.3: voxeilzation for non-watertight meshes.
run('./assignment1/run1 bunny_with_hole.stl 2.0 approx')
run('mv bunny_with_hole_voxel.stl ../data/assignment1/')
run('./assignment1/run1 spot_with_hole.stl 0.125 approx')
run('mv spot_with_hole_voxel.stl ../data/assignment1/')

# Assignment 1.4: marching cubes.
run('./assignment1/run1 bunny_voxel_info.txt')
run('./assignment1/run1 fandisk_voxel_info.txt')
run('./assignment1/run1 dragon_voxel_info.txt')
run('./assignment1/run1 spot_voxel_info.txt')
