import os
import numpy as np

# create folder for the meshes
if not os.path.exists("assn5_meshes"):
    os.mkdir("assn5_meshes")

# generate .scad
for radius in np.linspace(3.0, 4.0, num=11, endpoint=True):
    for offset in np.linspace(-3.0, -2.0, num=11, endpoint=True):
        file = open("assn5.scad","w")

        file.write("$fn = 100;\n") 
        file.write("difference() {\n") 
        file.write("cube([10,5,5], center=true);\n") 
        file.write("rotate([90,0,0]) translate([0,{:.2f},0]) cylinder(10,{:.2f},{:.2f}, center=true);\n".format(offset, radius, radius)) 
        file.write("}\n") 
        file.close() 

        # output mesh
        os.system("openscad -o assn5_meshes/bridge_r_{:d}_o_{:d}.stl assn5.scad".format(int(radius*10), int(offset*10)))

