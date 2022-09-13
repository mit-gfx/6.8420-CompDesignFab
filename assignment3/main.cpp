#include "FabSlicer.hpp"
#include "GCodeConverter.hpp"
#include <iostream>
#include <type_traits>

using Vector3 = Eigen::Matrix<double, 3, 1>;

void slice_to_gcode(std::string stl_in, std::string gcode_out, 
		bool bruteforce = false, double dz = 0.4, double infill_dx = 1.5,
		Vector3 bed_min = Vector3(0,0,0), Vector3 bed_max = Vector3(220, 220, 100))
{
	mesh::TriMesh<double> tri_mesh(stl_in, 1.0);

	fab_translation::FabSlicer<double> fab(tri_mesh, bed_min, bed_max, dz, infill_dx);

	std::vector<std::vector<std::vector<Eigen::Vector3d>>> contour;
	std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> infill_edges;

	fab.RunTranslation(contour, infill_edges, bruteforce);

	fab_translation::GCodeConverter::ConvertToGCode(gcode_out, contour, infill_edges);
}

int main(int argc, char** argv) {
    
    std::cerr << "Welcome to Assignment 3" << std::endl;

	bool do_bruteforce = true;
	std::string model_file = std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/bunny.stl";
	std::string gcode_file_brute_force = std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/results/bunny_brute_force.gcode";
	std::string gcode_file = std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/results/bunny.gcode";

	if (argc > 1) {
		if (argv[2][0] == '1')
			do_bruteforce = true;
		else
			do_bruteforce = false;

		model_file = std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/" + argv[1] + ".stl";
		gcode_file_brute_force = std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/results/" + argv[1] + "_brute_force.gcode";
		gcode_file = std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/results/" + argv[1] + ".gcode";
	}

    // Part 2: Brute Force Slicing
    //     Implement triangle-plane intersection in Geometry/include/BasicGeometry.hpp
    //     Implement brute-force slicing in         FabTranslation/Include/FabSlicer.hpp
    //     Implement contour creation in            FabTranslation/Include/FabSlicer.hpp

	if (do_bruteforce) {
		std::cout << "Slicing with brute force" << std::endl;
		slice_to_gcode(model_file, gcode_file_brute_force, true);
	}

    // Part 3: Accelerated Slicing
    //     (Optional) Implement an interval tree in DataStructure/include/IntervalTree.hpp
    //     Implement Slicing_accelerated in FabTranslation/Include/FabSlicer.hpp
    //             (you should also modify the constructor to initialize your tree)
    //     If contour creation is a bottleneck, accellerate it as well.

	else {
		std::cout << "Slicing accelerated" << std::endl;
		slice_to_gcode(model_file, gcode_file);
	}

    return 0;
}

