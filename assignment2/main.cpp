#include "voxelizer.hpp"
#include "marching_cube.hpp"
#include <iostream>
#include <sstream>
#include <ctime>
#include <string>
#include <iomanip>

int run_all();

int main(int argc, char* argv[]) {
    // Usage:
    // Slow version:
    // ./assignment2/run2 bunny.stl 2.0
    // ./assignment2/run2 fandisk.stl 0.05
    // ./assignment2/run2 spot.stl 0.125
    // ./assignment2/run2 dragon.stl 0.05
    // Fast version:
    // ./assignment2/run2 bunny.stl 2.0 fast
    // ./assignment2/run2 fandisk.stl 0.05 fast
    // ./assignment2/run2 spot.stl 0.125 fast
    // ./assignment2/run2 dragon.stl 0.05 fast
    // Approximation:
    // ./assignment2/run2 bunny_with_hole.stl 2.0 approx
    // ./assignment2/run2 spot_with_whole.stl 0.125 approx
    // Marching cube version:
    // ./assignment2/run2 bunny_voxel_info.txt
    // ./assignment2/run2 dragon_voxel_info.txt
    // ./assignment2/run2 fandisk_voxel_info.txt
    // ./assignment2/run2 spot_voxel_info.txt
    
    std::cerr << "Welcome to Assignment 2" << std::endl;

    if (argc == 1) {
        run_all();
        return 0;
    }

    if (argc == 2) {
        // Marching cube version.
        const std::string info_file(argv[1]);
        mesh::MarchingCube<double> mc(std::string(PROJECT_SOURCE_DIR) + "/data/assignment2/" + info_file);
        mc.BuildMesh();
        const std::string name = info_file.substr(0, info_file.size() - std::string("_voxel_info.txt").size());
        mc.ExportMeshToFile(std::string(PROJECT_SOURCE_DIR) + "/data/assignment2/results/" + name + "_mc.stl");
        return 0;
    }

    int t0 = std::clock();
    const std::string stl_name(argv[1]);
    const double dx = std::stod(argv[2]);
    mesh::Voxelizer<double> voxelizer(std::string(PROJECT_SOURCE_DIR) + "/data/assignment2/" + stl_name, dx);
    int t1 = std::clock();
    std::cout << "load mesh success... " << (double)(t1 - t0) / 1000000.0 << " seconds." << std::endl;
    std::cout << "Bounding box: " << voxelizer.pmin().transpose() << ", " << voxelizer.pmax().transpose() << std::endl;
    std::cout << "Number of voxels: " << voxelizer.voxel_num().transpose() << std::endl;
    bool fast = false;
    if (argc == 3) {
        voxelizer.BasicVoxelization();
    } else {
        const std::string flag(argv[3]);
        if (flag == "fast") {
            voxelizer.AdvancedVoxelization();
            fast = true;
        }
        else if (flag == "approx")
            voxelizer.AdvancedVoxelizationWithApproximation();
        else {
            std::cout << "ERROR: unexpected flag" << std::endl;
            exit(0);
        }
    }
    std::cout << "Voxelization done..." << std::endl;
    // Export results to mesh.
    std::string result_dir = std::string(PROJECT_SOURCE_DIR) + "/data/assignment2/results/"; 
    std::string stl_prefix = stl_name.substr(0, stl_name.size() - 4);
    if (fast) {
        stl_prefix += "_fast";
    }
    const std::string voxel_file_name = result_dir + stl_prefix + "_voxel.stl";
    std::cout << "Saving results to " << voxel_file_name << std::endl;
    voxelizer.WriteVoxelToMesh(voxel_file_name);
    const std::string voxel_info_file_name = result_dir + stl_prefix + "_voxel_info.txt";
    std::cout << "Saving voxel info to " << voxel_info_file_name << std::endl;
    voxelizer.WriteVoxelToFile(voxel_info_file_name);
    std::cout << "Results saved..." << std::endl;
    std::cout << "-----------------------------------------------------------------------------" << std::endl;
    return 0;
}

std::string voxelize(std::string model, std::string dx, std::string alg = "basic") {
    int argc = alg.compare("basic") ? 4:3;
    char* argv[4];
    argv[0] = "./assignment2/run2";
    argv[1] = &model[0u];
    argv[2] = &dx[0u];
    argv[3] = &alg[0u];
    int t0 = std::clock();
    main(argc, argv);
    int t1 = std::clock();
    std::stringstream ts;
    ts << std::fixed << std::setprecision(2) << (double)(t1 - t0) / 1000000.0;
    std::cout << "Command voxelize " << model << " " << dx << " " << alg 
        << " finished in " << ts.str() << " seconds." << std::endl;
    return ts.str();
}

std::string marching_cubes(std::string model) {
    int argc = 2;
    char* argv[2];
    argv[0] = "./assignment2/run2";
    argv[1] = &model[0u];
    int t0 = std::clock();
    main(argc, argv);
    int t1 = std::clock();
    std::stringstream ts;
    ts << std::fixed << std::setprecision(2) << (double)(t1 - t0) / 1000000.0;
    std::cout << "Command marching_cubes " << model  
        << " finished in " << ts.str() << " seconds." << std::endl;
    return ts.str();
}

int run_all() {
    // Assignment 2.1: basic voxelization.
    std::string basic_bunny = voxelize("bunny.stl", "2.0");
    std::string basic_fandisk = voxelize("fandisk.stl", "0.05");
    std::string basic_spot = voxelize("spot.stl", "0.125");
    std::string basic_dragon = voxelize("dragon.stl", "0.05");

    // Assignment 2.2: fast voxelization.
    std::string fast_bunny = voxelize("bunny.stl", "2.0", "fast");
    std::string fast_fandisk = voxelize("fandisk.stl", "0.05", "fast");
    std::string fast_spot = voxelize("spot.stl", "0.125", "fast");
    std::string fast_dragon = voxelize("dragon.stl", "0.05", "fast");


    // Assignment 2.3: voxelization for non-watertight meshes.
    std::string nwt_bunny = voxelize("bunny_with_hole.stl", "2.0", "approx");
    std::string nwt_spot = voxelize("spot_with_hole.stl", "0.125", "approx");

    // Assignment 2.4:  marching cubes.
    std::string marching_bunny = marching_cubes("bunny_voxel_info.txt");
    std::string marching_fandisk = marching_cubes("fandisk_voxel_info.txt");
    std::string marching_dragon = marching_cubes("dragon_voxel_info.txt");
    std::string marching_spot = marching_cubes("spot_voxel_info.txt");

    std::stringstream table;

    table <<
    "\\begin{center}" << std::endl <<
    "\\begin{tabular}{ c|c|c|c|c|c }" << std::endl <<
    " \\hline" << std::endl <<
    " \\textbf{Test case} & \\textbf{Time (s)} & \\textbf{Test case} & " <<
      "\\textbf{Time (s)} & \\textbf{Test case} & \\textbf{Time(s)} \\\\" <<
      std::endl <<
    " \\hline" << std::endl <<
    " Bunny (basic) & " << basic_bunny << " & Bunny (fast) & " << fast_bunny <<
      " & Bunny\\_with\\_hole & " << nwt_bunny << " \\\\" << std::endl <<
    " Fandisk (basic) & " << basic_fandisk << " & Fandisk (fast) & " << 
      fast_fandisk << " & Spot\\_with\\_hole & " << nwt_spot << " \\\\" << 
      std::endl <<
    " Spot (basic) & " << basic_spot << " & Spot (fast) & " << fast_spot <<
      " & & \\\\" << std::endl <<
    " Dragon (basic) & " << basic_dragon << " & Dragon (fast) & " <<
      fast_dragon << " & & \\\\" << std::endl <<
    " \\hline" << std::endl <<
    "\\end{tabular}" << std::endl <<
    "\\end{center}" << std:: endl;

    std::cout << table.str();
    
    return 0;
}
