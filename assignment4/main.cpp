#include "linear_material.hpp"
#include "neohookean_material.hpp"
#include "deformable_body.hpp"
#include <stdio.h>
#include <iostream>
#include "poly_mesh.hpp"
#include "tetrahedral_mesh.hpp"
#include "tet_deformable_body.hpp"
#include "typedefs.hpp"
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>
#include <fstream>
#include <set>

const size_t num_x_vertices = 4; //number of nodes (vertices) in x dimension of our mesh
const size_t num_y_vertices = 2; //number of nodes (vertices) in y dimension of our mesh
const size_t num_z_vertices = 2; //number of nodes (vertices) in z dimension of our mesh
const double spacing = 0.025; //size of each edge, 5 cm

void write_facet(std::ofstream& file, Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2) {
    file << "facet normal 0 0 0\n";
    file << "outer loop\n";
    file << "vertex " + std::to_string(p0.x()) + " " + std::to_string(p0.y()) + " " + std::to_string(p0.z()) + "\n";
    file << "vertex " + std::to_string(p1.x()) + " " + std::to_string(p1.y()) + " " + std::to_string(p1.z()) + "\n";
    file << "vertex " + std::to_string(p2.x()) + " " + std::to_string(p2.y()) + " " + std::to_string(p2.z()) + "\n";
    file << "endloop\n";
    file << "endfacet\n";
}

void write_tet_mesh(const std::string& filename, const Eigen::Matrix<double, 3, Eigen::Dynamic>& vertices, const Eigen::Matrix<int, 4, Eigen::Dynamic>& elements) {

    std::ofstream file;
    file.open(filename, std::ios::out);

    file << "solid tet_mesh \n";

    for (size_t i = 0; i < elements.cols(); ++i) {
        write_facet(file, vertices.col(elements(0, i)), vertices.col(elements(1, i)), vertices.col(elements(2, i)));
        write_facet(file, vertices.col(elements(1, i)), vertices.col(elements(3, i)), vertices.col(elements(2, i)));
        write_facet(file, vertices.col(elements(1, i)), vertices.col(elements(0, i)), vertices.col(elements(3, i)));
        write_facet(file, vertices.col(elements(0, i)), vertices.col(elements(2, i)), vertices.col(elements(3, i)));
    }

    file << "endsolid";

    file.close();
}

void set_boundary_conditions(const materials::TetrahedralMesh<double> tet_mesh, 
    Eigen::VectorXd& f_ext, // trimes external forces vector, size of final dofs
    std::vector<bool>& index_mask // index_mask[i * 3 + j] = false if the coordinate j (0<=j<=2) of vertex i is removed from variale list
    ) {
    
    const size_t rows = tet_mesh.vertex().rows();
    const size_t cols = tet_mesh.vertex().cols();
    const size_t total_dim = rows * cols;

    index_mask.resize(total_dim);
    for (size_t i = 0;i < total_dim;i++)
        index_mask[i] = true;

    // fix the vertices on the left face and remove them from variable list
    for (size_t i = 0; i < tet_mesh.vertex().cols(); ++i) {
        if (tet_mesh.vertex()(0, i) <= spacing / 2.0) {
            for (size_t j = 0;j < 3;j++) {
                index_mask[i * 3 + j] = false;
            }
        }
    }

    // apply external forces on each nodes of bottom right region
    Eigen::VectorXd f_ext_full = Eigen::VectorXd::Zero(total_dim);
    for (size_t i = 0; i < tet_mesh.vertex().cols(); ++i) {
        if (tet_mesh.vertex()(0, i) > (num_x_vertices / 2 - 1) * spacing - spacing / 2.0 &&
            tet_mesh.vertex()(2, i) < spacing / 2.0) {
            f_ext_full(3 * i + 2) = -50.0;
        }
    }

    // trim f_ext_full
    std::vector<int> index_mapping(total_dim);
    int dofs = 0;
    for (size_t i = 0;i < total_dim;i++)
        if (index_mask[i])
            index_mapping[i] = dofs ++;
    f_ext = Eigen::VectorXd(dofs); // trimed external forces
    for (size_t i = 0;i < total_dim; i++)
        if (index_mask[i])
            f_ext[index_mapping[i]] = f_ext_full[i];
}

// compute stiffness matrix K for current configuration
void get_K_and_fe(const materials::TetrahedralMesh<double> tet_mesh, // the tet mesh with rest configuration
    const materials::TetDeformableBody<double> tet_def_body, 
    const Eigen::MatrixXd vertices,                                  // the vertices coordinates in current configuration (size 3 * N)
    const std::vector<bool>& index_mask,                             // the index mask, which obtained from set_boundary_conditions
    Eigen::SparseMatrix<double>& K,                                  // output: the stiffness matrix
    Eigen::VectorXd& force_elastic                                   // output: the elastic force in current configuration
    ) {
    
    const size_t rows = tet_mesh.vertex().rows();
    const size_t cols = tet_mesh.vertex().cols();
    const size_t total_dim = rows * cols;

    Eigen::SparseMatrix<double> K_full = tet_def_body.ComputeStiffnessMatrix(vertices);

    Eigen::SparseMatrix<double> regularizer(K_full.rows(), K_full.cols());
    regularizer.setIdentity();

    // add a regularization term on the diagonal
    K_full += regularizer * 1e-4;

    Eigen::Matrix3Xd force_elastic_full = tet_def_body.ComputeElasticForce(vertices);

    // trim K_full and force_elastic_full with vertex_mask
    std::vector<int> index_mapping(total_dim);
    int dofs = 0;
    for (size_t i = 0;i < total_dim;i++)
        if (index_mask[i])
            index_mapping[i] = dofs ++;

    force_elastic = Eigen::VectorXd::Zero(dofs);
    std::vector<Eigen::Triplet<double> > triplet_list;
    triplet_list.clear();
    for (int k = 0;k < K_full.outerSize();k++)
        for (Eigen::SparseMatrix<double>::InnerIterator it(K_full, k); it; ++it) {
            if (index_mask[it.row()] && index_mask[it.col()]) {
                triplet_list.push_back(Eigen::Triplet<double>(index_mapping[it.row()], index_mapping[it.col()], it.value()));
            }
        }
    K = Eigen::SparseMatrix<double>(dofs, dofs);
    K.setFromTriplets(triplet_list.begin(), triplet_list.end());

    for (size_t i = 0;i < total_dim;i++)
        if (index_mask[i])
            force_elastic[index_mapping[i]] = force_elastic_full(i % 3, i / 3);
}

void test_linear_material() {

    std::cerr << "------------------------- Linear Material Model --------------------------------" << std::endl;

    const int dim = 3;

    //youngs =  10^7, poisson ratio = 0.45 - similar to that of silicone rubber
    materials::LinearElasticityMaterial<dim, double> linear_elasticity_material(10000000, 0.45);

    const size_t num_vertices = num_x_vertices * num_y_vertices * num_z_vertices;

    materials::TetrahedralMesh<double> tet_mesh =
        materials::TetrahedralMeshCuboid<double>(Eigen::Vector3i(num_x_vertices, num_y_vertices, num_z_vertices), spacing);

    //What does our undeformed mesh look like?
    write_tet_mesh(std::string(PROJECT_SOURCE_DIR) + "/data/assignment4/mesh_rest.stl", tet_mesh.vertex(), tet_mesh.element());

    std::vector<bool> index_mask;
    Eigen::VectorXd f_ext;
    set_boundary_conditions(tet_mesh, f_ext, index_mask);

    materials::TetDeformableBody<double> tet_def_body(linear_elasticity_material, tet_mesh.vertex(), 0.4, tet_mesh);

    Eigen::SparseMatrix<double> K;
    Eigen::VectorXd fe;
    get_K_and_fe(tet_mesh, tet_def_body, tet_mesh.vertex(), index_mask, K, fe);
    
    // solve K*u = f_ext by eigen cg solver
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Upper> solver;
    solver.setMaxIterations(1000000); //just a large number

    std::cout << "Time to solve" << std::endl;
    Eigen::VectorXd u = solver.compute(K).solve(f_ext);

    //get the final positions and shove them in u
    Eigen::Matrix<double, 3, Eigen::Dynamic> x = tet_mesh.vertex();
    int cnt = 0;
    for (size_t i = 0;i < num_vertices; i++) 
        for (size_t j = 0;j < 3;j++) {
            if (index_mask[i * 3 + j]) {
                x(j, i) += u(cnt++);
            }
    }

    // output K matrices and deformed mesh
    if (num_x_vertices == 4 && num_y_vertices == 2 && num_z_vertices == 2) {   // only output for 4x2x2
        std::string K_path = std::string(PROJECT_SOURCE_DIR) + "/data/assignment4/K_4x2x2_linear.txt";
        FILE* fp = fopen(K_path.c_str(), "w");
        for (size_t i = 0;i < K.rows();i++) {
            for (size_t j = 0;j < K.cols();j++)
                fprintf(fp, "%.6lf ", K.coeffRef(i, j));
            fprintf(fp, "\n");
        }
    }

    write_tet_mesh(std::string(PROJECT_SOURCE_DIR) + "/data/assignment4/deformed_linear.stl", x, tet_mesh.element());

    auto info = solver.info();
    std::string info_message;
    if (info == 0) {
        info_message = "Success";
    } else if (info == 1) {
        info_message = "NumericalIssue";
    } else if (info == 2) {
        info_message = "NoConvergence";
    } else if (info == 3) {
        info_message = "InvalidInput";
    } else {
        info_message = "UnknownError";
    }

    std::cout << "solve successful? " << info_message << std::endl;
}

// TODO: HW4
// Assignment 4, part 2.1
// Implement Newton's method to solve static equilibrium force for nonlinaer model.
// Hints:
//      1. Newton's method has four steps:
//          (1) Linearize the function at current position to make first-order approximation.
//          (2) Solve for the first-order approximation problem to get a optimization direction.
//          (3) Line search to find a step size in that direction
//          (4) Terminate when the solution is good enough.
void test_nonlinear_material() {
    std::cerr << "------------------------- Nonlinear Material Model --------------------------------" << std::endl;

    const int dim = 3;

    //youngs =  10^7, poisson ratio = 0.45 - similar to that of silicone rubber
    materials::NeohookeanElasticityMaterial<dim, double> nonlinear_elasticity_material(10000000, 0.45);

    const size_t num_vertices = num_x_vertices * num_y_vertices * num_z_vertices;

    materials::TetrahedralMesh<double> tet_mesh =
        materials::TetrahedralMeshCuboid<double>(Eigen::Vector3i(num_x_vertices, num_y_vertices, num_z_vertices), spacing);

    // set boundary conditions
    std::vector<bool> index_mask;
    Eigen::VectorXd f_ext;
    set_boundary_conditions(tet_mesh, f_ext, index_mask);

    materials::TetDeformableBody<double> tet_def_body(nonlinear_elasticity_material, tet_mesh.vertex(), 0.4, tet_mesh);

    // get your initial positions
    Eigen::MatrixXd vertices = tet_mesh.vertex();

    Eigen::SparseMatrix<double> K;
    Eigen::VectorXd fe;
    bool recompute_K_and_fe = true;

    // Newton's method
    // set max number of iterations of Newton's method to 1000
    int max_iters = 1000;
    for (int iter = 0;iter < max_iters;iter ++) {
        
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Upper> solver;
        solver.setMaxIterations(1000000); //just a large number

        /* Implement your code here */

        // Step 1: linearize the function around current solution vertices to get K and fe

        // Step 2: solve K*u = fe + f_ext

        // Step 3: line search for step size, so that |fe(vertices + u * step) + f_ext| < |fe(vertices) + f_ext|

        // Step 4: update vertices = vertices + u * step

        // Step 5: terminate Newton's method if |fe(vertices + u * step) + f_ext| < eps
    }

    // validate results
    get_K_and_fe(tet_mesh, tet_def_body, vertices, index_mask, K, fe);
    Eigen::VectorXd f_residual = f_ext + fe;
    double error = f_residual.norm();
    if (error < 1e-6) {
        std::cerr << "solve successful? Success" << std::endl;
    } else {
        std::cerr << "solve successful? Failed, Error = " << error << std::endl;
    }

    write_tet_mesh(std::string(PROJECT_SOURCE_DIR) + "/data/assignment4/deformed_nonlinear.stl", vertices, tet_mesh.element());
}

// TODO: HW4
// Assignment 4, part 3.1
// Write code to read a tetrahedral mesh in assignment 3, apply your customized boundary conditions
// Hints:
//  1. You can also take any other tetrahedral mesh from online.
//  2. If it takes very long to compute K matrix, to check:
//      (1) If K is a sparse matrix everywhere in your code
//      (2) If your tet mesh has too many vertices
void set_customized_boundary_conditions(const materials::TetrahedralMesh<double> tet_mesh, 
    Eigen::VectorXd& f_ext, // trimes external forces vector, size of final dofs
    std::vector<bool>& index_mask // index_mask[i * 3 + j] = false if the coordinate j (0<=j<=2) of vertex i is removed from variale list
    ) {
    /* Implement your code here */
}

void test_customized_model() {
    /* Implement your code here */
}

int main(int argc, char *argv[])
{
    std::cout << "Welcome to Assignment 4!" << std::endl;

    test_linear_material();

    test_nonlinear_material();
    
    test_customized_model();
    
    return 0;
}