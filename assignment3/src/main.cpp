#include "linear_blend_skinning_ui.hpp"
#include "handles.hpp"
#include <string>

void test(std::string testcase) {
	std::cerr << "running for  " << testcase << std::endl;

	std::string stl_filename = std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/BBW/" + testcase + ".stl";
	std::string handle_filename = std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/BBW/" + testcase + "_handles.txt";
	
	// load stl
	Eigen::MatrixXd V_temp, V_temp_unique;
	Eigen::MatrixXi F_temp;
	Eigen::VectorXi SVI, SVJ;
	igl::read_triangle_mesh(stl_filename, V_temp, F_temp);
	igl::remove_duplicate_vertices(V_temp, 0, V_temp_unique, SVI, SVJ);
	std::for_each(F_temp.data(), F_temp.data() + F_temp.size(), [&SVJ](int& f) {f = SVJ(f); });
	V_temp = V_temp_unique;

	if (F_temp.rows() > LinearBlendSkinningUI::MAX_FACES) {
		std::cout << "Mesh too large, decimating." << std::endl;
		Eigen::MatrixXd V_dec_temp;
		Eigen::MatrixXi F_dec_temp;
		Eigen::VectorXi J, I;

		std::cout << "Size before = " << V_temp.rows() << " x " << V_temp.cols() << std::endl;

		if (igl::decimate(V_temp, F_temp, LinearBlendSkinningUI::MAX_FACES, V_dec_temp, F_dec_temp, J, I)) {
			std::cout << "Size after = " << V_dec_temp.rows() << " x " << V_dec_temp.cols() << std::endl;
			V_temp = V_dec_temp;
			F_temp = F_dec_temp;
		}
		else {
			std::cout << "Decimation failed" << std::endl;
		}
	}

	Eigen::MatrixXd V; // Vertex Positions
	Eigen::MatrixXi T; // Tetrahedral Elements
	Eigen::MatrixXi F; // Triangular Faces of exterior surface
	if (igl::copyleft::tetgen::tetrahedralize(V_temp, F_temp, "pq1.414", V, T, F) == 0) {
		F.col(0).swap(F.col(2));
	}
	else {
		std::cout << "Failed to tetrahedralize mesh." << std::endl;
	}

	// load handles
	WeightHandles handles;
	handles.load_handle_file(handle_filename);

	// compute bounded biharmonic weights
	Eigen::VectorXi b;
	Eigen::MatrixXd bc;
	Eigen::MatrixXd W;
	handles.boundary_conditions(V, T, b, bc);
	bounded_biharmonic_weights(V, T, b, bc, W);

	Eigen::MatrixXd lbs_mat;
	igl::lbs_matrix(V, W, lbs_mat);

	// output to file
	std::string output_filename = std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/BBW/results/" + testcase + "_weights.txt";
	std::ofstream f(output_filename);
	f << lbs_mat;
	f.close();
}

void run_all() {
	test("frame");
	test("spot");
	test("tyra");
	test("bunny");

	// std::cerr << "finished. please to into the results folder and run [python grade.py] to check your results. " << std::endl;
}

int main(int argc, char** argv) {
	if (argc > 1 && std::string(argv[1]) == "test") {
		run_all();
	} else {
		igl::opengl::glfw::Viewer viewer;
		LinearBlendSkinningUI ui;
		viewer.plugins.push_back(&ui);
		viewer.launch();
	}

	return 0;
}