#pragma once

#include <igl/boundary_conditions.h>
#include <Eigen/Core>
#include <fstream>
#include <igl/project.h>
#include <igl/lbs_matrix.h>
#include <limits>

struct WeightHandles {
public:
	WeightHandles() :
		handle_positions(0, 3),
		point_handle_indices(0),
		bone_edge_indices(0, 2),
		cage_edge_points(0, 2)
	{
		// Nothing to do
	}

	Eigen::MatrixXd& positions() {
		return handle_positions;
	}

	void load_handle_file(std::string filename)
	{
		std::ifstream file(filename);
		read_mat(handle_positions, file);
		read_vec(point_handle_indices, file);
		read_mat(bone_edge_indices, file);
		read_mat(cage_edge_points, file);
		file.close();

		handle_transforms.resize(4 * handle_positions.rows(), 3);
		reset_handle_transformations();
		int n = handle_positions.rows();
		igl::lbs_matrix(handle_positions, Eigen::MatrixXd::Identity(n, n), handle_lbs_matrix);
	}

	void save_handle_file(std::string filename)
	{
		std::ofstream file(filename);
		write_mat(handle_positions, file);
		write_vec(point_handle_indices, file);
		write_mat(bone_edge_indices, file);
		write_mat(cage_edge_points, file);
		file.close();
	}

	
	void add_point_handle(const Eigen::Vector3d point)
	{
		int n = handle_positions.rows();
		int m = point_handle_indices.rows();
		handle_positions.conservativeResize(n + 1, Eigen::NoChange);
		handle_positions.row(n) = point;
		point_handle_indices.conservativeResize(m + 1, Eigen::NoChange);
		point_handle_indices(m) = n;

		handle_transforms.conservativeResize(4 * handle_positions.rows(), 3);
		handle_transforms.block(4 * n, 0, 4, 3) <<
			1, 0, 0,
			0, 1, 0,
			0, 0, 1,
			0, 0, 0;
		n = handle_positions.rows();
		igl::lbs_matrix(handle_positions, Eigen::MatrixXd::Identity(n, n), handle_lbs_matrix);
	}

	void add_bone_edge(const Eigen::Vector3d& tip, const Eigen::Vector3d& tail)
	{
		int n = handle_positions.rows();
		int m = bone_edge_indices.rows();
		handle_positions.conservativeResize(n + 2, Eigen::NoChange);
		handle_positions.row(n) = tip;
		handle_positions.row(n + 1) = tail;
		bone_edge_indices.conservativeResize(m + 1, Eigen::NoChange);
		bone_edge_indices(m, 0) = n;
		bone_edge_indices(m, 1) = n + 1;

		handle_transforms.conservativeResize(4 * handle_positions.rows(), 3);

		handle_transforms.block(4 * n, 0, 4, 3) <<
			1, 0, 0,
			0, 1, 0,
			0, 0, 1,
			0, 0, 0;

		handle_transforms.block(4 * (n+1), 0, 4, 3) <<
			1, 0, 0,
			0, 1, 0,
			0, 0, 1,
			0, 0, 0;
		n = handle_positions.rows();
		igl::lbs_matrix(handle_positions, Eigen::MatrixXd::Identity(n, n), handle_lbs_matrix);
	}

	void add_cage_edge(const Eigen::Vector3d& tip, const Eigen::Vector3d& tail)
	{
		int n = point_handle_indices.rows();
		int m = cage_edge_points.rows();
		add_point_handle(tip);
		add_point_handle(tail);
		cage_edge_points.conservativeResize(m + 1, Eigen::NoChange);
		cage_edge_points(m, 0) = n;
		cage_edge_points(m, 1) = n + 1;
	}

	bool boundary_conditions(
		const Eigen::MatrixXd& V, 
		const Eigen::MatrixXi& F, 
		Eigen::VectorXi& b, 
		Eigen::MatrixXd& bc) 
	{
		return igl::boundary_conditions(
			V,
			F,
			handle_positions,
			point_handle_indices,
			bone_edge_indices,
			cage_edge_points,
			b,
			bc
		);
	}

	// Get the Nearest Handle within
	int select_handle(
		const Eigen::Vector2f& pos,
		const Eigen::Matrix4f& model,
		const Eigen::Matrix4f& proj,
		const Eigen::Vector4f& viewport,
		double threshold = std::numeric_limits<double>::max()) 
	{
		
		Eigen::Vector3f s, dir;
		Eigen::Vector3d src, dst;
		
		igl::unproject_ray(pos, model, proj, viewport, s, dir);
		
		src = s.cast<double>();
		dst = s.cast<double>() + dir.cast<double>();
		int closest = -1;
		float best = threshold*threshold;
		for (int i = 0; i < handle_positions.rows(); ++i) {
			double t, sqrd;
			igl::project_to_line(
				handle_positions(i, 0), handle_positions(i, 1), handle_positions(i, 2),
				src(0), src(1), src(2),
				dst(0), dst(1), dst(2),
				t, sqrd
			);
			if (sqrd < best) {
				best = sqrd;
				closest = i;
			}
		}

		return closest;
	}

	void reset_handle_transformations() {
		handle_transforms.resize(4 * handle_positions.rows(), 3);
		for (int i = 0; i < handle_positions.rows(); ++i) {
			handle_transforms.block(4*i, 0, 4, 3) <<
				1, 0, 0,
				0, 1, 0,
				0, 0, 1,
				0, 0, 0;
		}
	}

	void move_handle(
		int handle_id,
		const Eigen::RowVector3d pos)
	{
		Eigen::RowVector3d old_pos = handle_positions.row(handle_id);
		Eigen::RowVector3d translation = pos - old_pos;
		handle_transforms.block(4 * handle_id + 3, 0, 1, 3) = translation;
	}

	Eigen::MatrixXd& transform() {
		return handle_transforms;
	}

	void transfored_handles(Eigen::MatrixXd& H) {
		H = handle_lbs_matrix * handle_transforms;
	}

	void visualize_handles(
		Eigen::MatrixXd& points, 
		Eigen::MatrixXd& point_colors, 
		Eigen::MatrixXi& lines, 
		Eigen::MatrixXd& line_colors,
		bool transform = false)
	{

		Eigen::Vector3d point_handle_color(1.0, 0.0, 0.0); // Red
		Eigen::Vector3d bone_handle_color(0.0, 1.0, 0.0); // Green
		Eigen::Vector3d cage_handle_color(0.0, 0.0, 1.0); // Blue

		if (transform) {
			points = handle_lbs_matrix * handle_transforms;
		}
		else {
			points = handle_positions;
		}
		point_colors.resizeLike(points);
		lines.resize(bone_edge_indices.rows() + cage_edge_points.size(), 2);
		lines.block(0, 0, bone_edge_indices.rows(), 2) = bone_edge_indices;
		line_colors.resize(lines.rows(), 3);

		for (int i = 0; i < point_handle_indices.size(); ++i) {
			point_colors.row(point_handle_indices(i)) = point_handle_color;
		}
		for (int i = 0; i < bone_edge_indices.rows(); ++i) {
			point_colors.row(bone_edge_indices(i, 0)) = bone_handle_color;
			point_colors.row(bone_edge_indices(i, 1)) = bone_handle_color;
			line_colors.row(i) = bone_handle_color;
		}
		for (int i = 0; i < cage_edge_points.rows(); ++i) {
			int u = point_handle_indices(cage_edge_points(i, 0));
			int v = point_handle_indices(cage_edge_points(i, 1));
			point_colors.row(u) = cage_handle_color;
			point_colors.row(v) = cage_handle_color;
			int line_row = bone_edge_indices.rows() + i;
			lines.row(line_row) = Eigen::Vector2i(u, v);
			line_colors.row(line_row) = cage_handle_color;
		}
	}

private:

	template <typename T>
	void read_mat(Eigen::Matrix<T, -1, -1>& mat, std::ifstream& file) {
		int n, m;
		T v;
		file >> n;
		file >> m;
		mat.resize(n, m);
		for (int i = 0; i < mat.rows(); ++i) {
			for (int j = 0; j < mat.cols(); ++j) {
				file >> v;
				mat(i, j) = v;
			}
		}
	}

	template <typename T>
	void write_mat(const Eigen::Matrix<T, -1, -1> & mat, std::ofstream& file) {
		file << mat.rows() << " ";
		file << mat.cols() << " ";
		for (int i = 0; i < mat.rows(); ++i) {
			for (int j = 0; j < mat.cols(); ++j) {
				file << mat(i, j) << " ";
			}
		}
	}

	void read_vec(Eigen::VectorXi& vec, std::ifstream& file) {
		int n;
		int v;
		file >> n;
		vec.resize(n);
		for (int i = 0; i < vec.size(); ++i) {
			file >> v;
			vec(i) = v;
		}
	}

	void write_vec(const Eigen::VectorXi& vec, std::ofstream& file) {
		file << vec.size() << " ";
		for (int i = 0; i < vec.size(); ++i) {
			file << vec(i) << " ";
		}
	}

	Eigen::MatrixXd handle_transforms;
	Eigen::MatrixXd handle_lbs_matrix;

	Eigen::MatrixXd handle_positions;
	Eigen::VectorXi point_handle_indices; // indices into handle_positions
	Eigen::MatrixXi bone_edge_indices;    // indices into handle_positions
	Eigen::MatrixXi cage_edge_points;     // indices into point_handle_indices

};