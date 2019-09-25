import os
import sys
import time

def run(command):
    t0 = time.time()
    os.system(command)
    t1 = time.time()
    print('Command: %s finished in %f seconds...' % (command, t1 - t0))

# Assignment 2.1: basic voxelization.
run('./assignment2/run2 bunny.stl 2.0')
run('mv bunny_voxel.stl ../data/assignment2/')
run('./assignment2/run2 fandisk.stl 0.05')
run('mv fandisk_voxel.stl ../data/assignment2/')
run('./assignment2/run2 spot.stl 0.125')
run('mv spot_voxel.stl ../data/assignment2/')
run('./assignment2/run2 dragon.stl 0.05')
run('mv dragon_voxel.stl ../data/assignment2/')

# Assignment 2.2: fast voxelization.
run('./assignment2/run2 bunny.stl 2.0 fast')
run('mv bunny_voxel.stl ../data/assignment2/bunny_voxel_fast.stl')
run('mv bunny_voxel_info.txt ../data/assignment2/assignment2/')
run('./assignment2/run2 fandisk.stl 0.05 fast')
run('mv fandisk_voxel.stl ../data/assignment2/fandisk_voxel_fast.stl')
run('mv fandisk_voxel_info.txt ../data/assignment2/')
run('./assignment2/run2 spot.stl 0.125 fast')
run('mv spot_voxel.stl ../data/assignment2/spot_voxel_fast.stl')
run('mv spot_voxel_info.txt ../data/assignment2/')
run('./assignment2/run2 dragon.stl 0.05 fast')
run('mv dragon_voxel.stl ../data/assignment2/dragon_voxel_fast.stl')
run('mv dragon_voxel_info.txt ../data/assignment2/')

# Assignment 2.3: voxeilzation for non-watertight meshes.
run('./assignment2/run2 bunny_with_hole.stl 2.0 approx')
run('mv bunny_with_hole_voxel.stl ../data/assignment2/')
run('./assignment2/run2 spot_with_hole.stl 0.125 approx')
run('mv spot_with_hole_voxel.stl ../data/assignment2/')

# Assignment 2.4: marching cubes.
run('./assignment2/run2 bunny_voxel_info.txt')
run('./assignment2/run2 fandisk_voxel_info.txt')
run('./assignment2/run2 dragon_voxel_info.txt')
run('./assignment2/run2 spot_voxel_info.txt')
