#pragma once

#define IGL_VIEWER_VIEWER_QUIET 1
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/lbs_matrix.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/unproject_in_mesh.h>
#include <igl/read_triangle_mesh.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/decimate.h>

#include "handles.hpp"
#include "mesh_util.hpp"
#include "nearest_neighbor_weights.hpp"
#include "bounded_biharmonic_weights.hpp"
#include "linear_weights.hpp"


class LinearBlendSkinningUI : public igl::opengl::glfw::ViewerPlugin {
public:
	LinearBlendSkinningUI() :
		create_handle_mode(false),
		weight_type(0),
		menu()
	{
		// Nothing to do
	}

	void init(igl::opengl::glfw::Viewer* _viewer) {
		igl::opengl::glfw::ViewerPlugin::init(_viewer);
		viewer->core().set_rotation_type(igl::opengl::ViewerCore::RotationType::ROTATION_TYPE_TRACKBALL);

		viewer->plugins.push_back(&menu);
		menu.callback_draw_viewer_menu = [&]() {draw_menu(); };

	}

	void draw_menu() {

		// Helper for making checkboxes
		auto make_checkbox = [&](const char* label, unsigned int& option)
		{
			return ImGui::Checkbox(label,
				[&]() { return viewer->core().is_set(option); },
				[&](bool value) { return viewer->core().set(option, value); }
			);
		};

		if (ImGui::Button("Load Voxel File")) {
			auto filename = igl::file_dialog_open();
			if (filename.size() > 0) {
				auto hex_mesh = materials::vox2hex<double>(filename);
				auto tet_mesh = materials::hex2tet(hex_mesh);
				V = tet_mesh.vertex().transpose();
				T = tet_mesh.element().transpose();
				igl::boundary_facets(T, F);
				F.col(0).swap(F.col(2));
					
				viewer->data().clear();
				viewer->data().set_mesh(V, F);
				viewer->core().align_camera_center(viewer->data().V, viewer->data().F);
				handles = WeightHandles();
				create_handle_mode = 0;
			}
		}

		if (ImGui::Button("Load Triangle Mesh")) {
			auto filename = igl::file_dialog_open();
			if (filename.size() > 0) {
				Eigen::MatrixXd V_temp, V_temp_unique;
				Eigen::MatrixXi F_temp;
				Eigen::VectorXi SVI, SVJ;
				igl::read_triangle_mesh(filename, V_temp, F_temp);
				igl::remove_duplicate_vertices(V_temp, 0, V_temp_unique, SVI, SVJ);
				std::for_each(F_temp.data(), F_temp.data() + F_temp.size(), [&SVJ](int& f) {f = SVJ(f); });
				V_temp = V_temp_unique;

				if (F_temp.rows() > MAX_FACES) {
					std::cout << "Mesh too large, decimating." << std::endl;
					Eigen::MatrixXd V_dec_temp;
					Eigen::MatrixXi F_dec_temp;
					Eigen::VectorXi J, I;

					std::cout << "Size before = " << V_temp.rows() << " x " << V_temp.cols() << std::endl;

					if (igl::decimate(V_temp, F_temp, MAX_FACES, V_dec_temp, F_dec_temp, J, I)) {
						std::cout << "Size after = " << V_dec_temp.rows() << " x " << V_dec_temp.cols() << std::endl;
						V_temp = V_dec_temp;
						F_temp = F_dec_temp;
					}
					else {
						std::cout << "Decimation failed" << std::endl;
					}
				}

				if (igl::copyleft::tetgen::tetrahedralize(V_temp, F_temp, "pq1.414", V, T, F) == 0) {
					F.col(0).swap(F.col(2));

					viewer->data().clear();
					viewer->data().set_mesh(V, F);
					viewer->core().align_camera_center(viewer->data().V, viewer->data().F);
					handles = WeightHandles();
					create_handle_mode = 0;
				}
				else {
					std::cout << "Failed to tetrahedralize mesh." << std::endl;
				}
				std::cerr << "number of vertices = " << V.rows() << std::endl;
				std::cerr << "number of faces = " << T.rows() << std::endl;
			}
		}

		// Only Allow Mesh Manipulations if we have a mesh
		if (V.rows() > 0) {

			ImGui::NewLine();

			if (ImGui::Button("Load Handles")) {
				auto filename = igl::file_dialog_open();
				if (filename.size() > 0) {
					handles.load_handle_file(filename);
					draw_handles();
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Save Handles")) {
				auto filename = igl::file_dialog_save();
				if (filename.size() > 0) {
					handles.save_handle_file(filename);
				}
			}

			ImGui::NewLine();
			ImGui::Text("Handle Mode");
			if (ImGui::RadioButton("Create", &create_handle_mode, 0)) {
				handles.reset_handle_transformations();
				viewer->data().set_vertices(V);
				draw_handles();
			}
			// Only allow handle manipulation when there are handles to manipulate
			if (handles.positions().rows() > 0) {
				ImGui::SameLine();
				if (ImGui::RadioButton("Manipulate", &create_handle_mode, 1)) {
					if (create_handle_mode == 1) {
						std::cout << "Recomputing Weights - UI may hang during this." << std::endl;
						compute_weights();
					}
				}
			}

			ImGui::NewLine();
			ImGui::Text("Weight Type");
			int last_weight_type = weight_type;
			ImGui::RadioButton("Nearest Neighbor", &weight_type, 0);
			ImGui::RadioButton("Linear", &weight_type, 1);
			ImGui::RadioButton("Bounded Biharmonic", &weight_type, 2);
			if (last_weight_type != weight_type && create_handle_mode == 1) {
				compute_weights();
				if (handles.positions().rows() > 0) {
					viewer->data().set_vertices(lbs_mat * handles.transform());
				}
			}

			ImGui::NewLine();

			if (ImGui::CollapsingHeader("Viewer Options")) {
				make_checkbox("Wireframe", viewer->data().show_lines);
				make_checkbox("Fill", viewer->data().show_faces);
			}

			ImGui::NewLine();

			if (ImGui::CollapsingHeader("Debug")) {
				if (ImGui::Button("Laplacian")) {
					auto filename = igl::file_dialog_save();
					if (filename.size() > 0) {
						Eigen::SparseMatrix<double> L;
						cotangent_laplacian(V, T, L);
						std::ofstream f(filename);
						f << L;
						f.close();
					}
				}
				if (handles.positions().rows() > 0) {
					if (ImGui::Button("Weights")) {
						auto filename = igl::file_dialog_save();
						if (filename.size() > 0) {
							compute_weights();
							std::ofstream f(filename);
							f << lbs_mat;
							f.close();
						}
					}
				}
			}
		}
	}

	void draw_handles() {
		Eigen::MatrixXd points, point_colors, line_colors;
		Eigen::MatrixXi lines;
		handles.visualize_handles(points, point_colors, lines, line_colors, create_handle_mode == 1);
		if (moving_handle_id >= 0) {
			point_colors.row(moving_handle_id) = Eigen::Vector3d(1.0, 0.7, 0.2);
		}
		viewer->data().clear_labels();
		viewer->data().set_points(points, point_colors);
		viewer->data().set_edges(points, lines, line_colors);
	}

	void compute_weights() {
		// Only compute weights if there are weights to compute!
		if (handles.positions().rows() > 0) {
			Eigen::MatrixXd W;
			switch (weight_type) {
			case 0: // Nearest Neighbor
				ComputeNearestNeighborWeights(V, handles.positions(), W);
				igl::normalize_row_sums(W, W);
				break;
			case 1: // Linear
				ComputeLinearSkinningWeights(V, handles.positions(), W);
				igl::normalize_row_sums(W, W);
				break;
			case 2: // Bounded Biharmonic Weights
				Eigen::VectorXi b;
				Eigen::MatrixXd bc;
				handles.boundary_conditions(V, T, b, bc);
				bounded_biharmonic_weights(V, T, b, bc, W);
				break;
			}

			igl::lbs_matrix(V, W, lbs_mat);
		}
	}

	bool mouse_down(int button, int modifier) {
		if (button == 0 && create_handle_mode != 0) {
			int v = get_closest_mesh_vertex();
			if (v >= 0) {
				Eigen::RowVector3d pos = viewer->data().V.row(v);
				Eigen::MatrixXd H;
				handles.transfored_handles(H);
				int best = 0;
				for (int i = 0; i < H.rows(); ++i) {
					Eigen::RowVector3d h = H.row(i);
					if ((h - pos).norm() < (Eigen::RowVector3d(H.row(best)) - pos).norm()) {
						best = i;
					}
				}
				moving_handle_id = best;
				sel_pos = H.row(best);
				draw_handles();
				return true;
			}
		}
		return false;
	}

	bool mouse_up(int button, int modifier)
	{
		moving_handle_id = -1;
		draw_handles();
		// Check to see if click or rotation
		double dx = viewer->current_mouse_x - viewer->down_mouse_x;
		double dy = viewer->current_mouse_y - viewer->down_mouse_y;
		double dist_sqr = dx * dx + dy * dy;
		if (dist_sqr > 2.0) {
			return false;
		}

		// Left-Click to Add a Handle
		if (button == 0)
		{
			if (viewer->data().V.rows() > 0 && viewer->data().F.rows() > 0 && create_handle_mode == 0) {
				int v = get_closest_mesh_vertex();
				if (v >= 0) {
					Eigen::Vector3d pos = viewer->data().V.row(v);
					handles.add_point_handle(pos);
					draw_handles();
					return true;
				}
			}
		}
		return false;
	}

	bool mouse_move(int mouse_x, int mouse_y) {
		if (create_handle_mode == 1 && moving_handle_id >= 0) {

			float x = viewer->current_mouse_x;
			float y = viewer->core().viewport(3) - viewer->current_mouse_y;

			Eigen::RowVector3f orig_pos = sel_pos.cast<float>();

			Eigen::RowVector3f orig_screen_pos;

			igl::project(
				orig_pos,
				viewer->core().view,
				viewer->core().proj,
				viewer->core().viewport,
				orig_screen_pos
			);

			Eigen::RowVector3f new_screen_pos((float)x, (float)y, orig_screen_pos(2));
			Eigen::RowVector3f new_pos;

			igl::unproject(
				new_screen_pos,
				viewer->core().view,
				viewer->core().proj,
				viewer->core().viewport,
				new_pos
			);

			Eigen::RowVector3d pos = new_pos.cast<double>();

			handles.move_handle(
				moving_handle_id,
				pos);
			
			
			viewer->data().set_vertices(lbs_mat * handles.transform());
			draw_handles();
			return true;
		}
		return false;
	}

	int get_closest_mesh_vertex() {
		int index = -1;
		int fid;
		Eigen::Vector3f bc;
		double x = viewer->current_mouse_x;
		double y = viewer->core().viewport(3) - viewer->current_mouse_y;
		Eigen::RowVector3f last_mouse(x, y, 0);
		if (igl::unproject_onto_mesh(
			last_mouse.head(2),
			viewer->core().view,
			viewer->core().proj,
			viewer->core().viewport,
			viewer->data().V,
			viewer->data().F,
			fid,
			bc))
		{
			const Eigen::MatrixXi& F = viewer->data().F;
			int coord;
			bc.maxCoeff(&coord);
			index = F(fid, coord);
		}
		return index;
	}

	static const int MAX_FACES = 10000;

private:

	igl::opengl::glfw::imgui::ImGuiMenu menu;

	Eigen::MatrixXd V; // Vertex Positions
	Eigen::MatrixXi T; // Tetrahedral Elements
	Eigen::MatrixXi F; // Triangular Faces of exterior surface

	int weight_type = 0; // Type of weights to compute
	                     // 0 = Nearest Neighbor
	                     // 1 = Linear
	                     // 2 = Bounded Biharmonic

	Eigen::MatrixXd lbs_mat; // Linear Blend Skinning Matrix


	// Handle Creation State
	int create_handle_mode = 0; // 0 = add_handle, 1 = manipulate handles
	WeightHandles handles;


	// Handle Manipulation State
	int moving_handle_id = -1;
	Eigen::RowVector3d sel_pos; // Position of handle when it was selected
};