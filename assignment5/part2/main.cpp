// include FEM
#include "poly_mesh.hpp"
#include "linear_material.hpp"
#include "deformable_body.hpp"
#include "tetrahedral_mesh.hpp"
#include "tet_deformable_body.hpp"
#include "typedefs.hpp"
// include Mesh
#include "voxelizer.hpp"
#include "marching_cube.hpp"
// include Geometry
#include "GeometryExploration.hpp"
// include Eigen
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
// include std
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <set>

// compute the performance metrics for a given model stored in stl_name
// input: 
//  stl_name: the filename for the model
// output:
//  compliance: a scalar value for the compliance (L = F'U = U'*K*U)
//  num_voxels: a scalar value for the number of solid voxels
void SolvePerformance(std::string stl_name, double& compliance, double& num_voxels) {
    // TODO: HW5
    // part 2.2 implement the pipeline from design space to the performance space
    // compute the compliance and the num_voxels for the model
    const int dim = 3;

    // using this linear material
    materials::LinearElasticityMaterial<dim, double> linear_elasticity_material(10000000, 0.45);
    double dx = 0.25; // spacing parameter(need to be fixed)
    double density = 1.0;

    // step 1: voxelize the mesh
    mesh::Voxelizer<double> voxelizer(stl_name, dx);
    voxelizer.AdvancedVoxelization();

    // step 2: convert the voxelization results to tetrahedral mesh

    // step 3: construct boundary conditions for fem

    // step 4: compute compliance by the fem implemented in assignment 4, please use the linear model we pre-defined

    // step 5: compute num_voxels
}

int main(int argc, char *argv[])
{ 
    std::cerr << "Welcome to Assignment 5" << std::endl;
    
    int N = 200000;
    std::vector<Eigen::Vector2d> p1_input; 

    p1_input.clear();

    // fix random seed to get repeatable results
    srand (1);

    for (int i = 0; i < N; i++) {
        p1_input.push_back(Eigen::Vector2d(i + 1, N - i) + Eigen::Vector2d::Random() * 2.0);
    }

    // test 2.1: fast 2d Pareto front
    std::vector<Eigen::Vector2d> p1_result = geometry::ParetoFront2D(p1_input);
    // print 2.1 results
    std::ofstream file1;
    file1.open(PROJECT_SOURCE_DIR"/data/assignment5/q1_result.txt");
    file1 << "print 2.1 test result" << std::endl;
    file1 << "Totol number of points: " << p1_result.size() << std::endl;
    for (int i = 0; i < p1_result.size(); i++) {
        file1 << "P" << i << ": " << std::endl;
        file1 << p1_result[i] << std::endl;
    }
    file1.close();

    // 2.2: implement the pipeline from design space to the performance space
    std::vector<Eigen::Vector2d> sample_performance;
    
    double compliance, num_voxels;
    
    // TODO: HW5
    // part 2.2 mapping from design space to performance space
    // Once you debug the bridge example correct, comment the code before dash line
    // and uncomment the code after dash line to run the test on 121 bridges
    SolvePerformance(PROJECT_SOURCE_DIR"/data/assignment5/bridge.stl", compliance, num_voxels);
    SolvePerformance(PROJECT_SOURCE_DIR"/data/assignment5/CSG/assn5_meshes/bridge_r_40_o_-25.stl", compliance, num_voxels);
    std::cout << compliance << " " << num_voxels << std::endl;
    // -----------------------------------------------------------------------
    // std::string base(PROJECT_SOURCE_DIR"/data/assignment5/CSG/assn6_meshes/bridge");
    // int radius_start = 30;
    // int radius_end   = 40;
    // int offset_start = -30;
    // int offset_end   = -20;

    // std::ofstream file2;
    // file2.open("q2_result.txt");
    // file2 << "print 2.2 test result" << std::endl << std::endl;

    // int count = 0;
    // for (int r = radius_start; r <= radius_end; r++) {
    //     for (int o = offset_start; o <= offset_end; o++) {
    //         std::string mesh_name = base + "_r_" + std::to_string(r) + "_o_" + std::to_string(o) + ".stl";
    //         std::cout << "bridge_r_" + std::to_string(r) + "_o_" + std::to_string(o) << std::endl;
    //         SolvePerformance(mesh_name, compliance, num_voxels);
    //         std::cerr << "compliance = " << compliance << ", num_voxels = " << num_voxels << std::endl;
    //         sample_performance.push_back(Eigen::Vector2d(compliance, num_voxels));
    //         file2 << "bridge_r_" + std::to_string(r) + "_o_" + std::to_string(o) + ".stl" << std::endl;
    //         file2 << "Compliance: " +  std::to_string(compliance) << std::endl;
    //         file2 << "Total mass: " + std::to_string(num_voxels) << std::endl << std::endl;
    //         count++;
    //     }
    // }
    // file2.close();

    // std::vector<Eigen::Vector2d> p2_result = geometry::ParetoFront2D(sample_performance);
    // // print Q3 results
    // file2.open("q2_pareto_front_result.txt");
    // file2 << "print 2.2 pareto front result" << std::endl;
    // file2 << "Totol number of points: " << p2_result.size() << std::endl;
    // for (int i = 0; i < p2_result.size(); i++) {
    //     file2 << "P" << i << ": " << std::endl;
    //     file2 << p2_result[i] << std::endl;
    // }
    // file2.close();

    return 0;
}
