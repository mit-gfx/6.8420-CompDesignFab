#pragma once

#include "read_stl.hpp"
#include "BasicGeometry.hpp"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <unordered_set>
#include <fstream>
#include <ctime>
#include <map>
#include <set>
#include "hexahedral_mesh.hpp"

namespace mesh {

	template<typename T>
	class Voxelizer {
	public:
		Voxelizer(const std::string& stl_file_name, const T dx)
			: _dx(dx) {
			// Randomness.
			srand(static_cast<unsigned>(time(0)));
			// Load triangles from the stl file.
			std::vector<Vector3<T>> normals;
			if (!ReadSTL(stl_file_name, _triangles, normals)) {
				std::cout << "ERROR: cannot read " << stl_file_name << std::endl;
				return;
			}
			// Compute the bounding box of _triangle and save the results into _pmin.
			_pmin = _triangles[0][0];
			Vector3<T> pmax = _triangles[0][0];
			for (const auto& triangle : _triangles)
				for (const auto& v : triangle) {
					_pmin = _pmin.cwiseMin(v);
					pmax = pmax.cwiseMax(v);
				}
			for (int i = 0; i < 3; ++i) {
				_pmin[i] -= _dx;
				pmax[i] += _dx;
			}
			// Compute the number of voxels along each direction.
			for (int i = 0; i < 3; ++i)
				_nvoxel[i] = static_cast<int>((pmax[i] - _pmin[i]) / _dx) + 1;
			// Initialize the voxel array.
			_voxels = std::vector<std::vector<std::vector<bool>>>(_nvoxel.x(),
				std::vector<std::vector<bool>>(_nvoxel.y(),
					std::vector<bool>(_nvoxel.z(), false)));
		}

		const Vector3<T> pmin() const { return _pmin; }
		const T dx() const { return _dx; }
		const Vector3<T> pmax() const { return _pmin + Vector3<T>(_nvoxel.x(), _nvoxel.y(), _nvoxel.z()) * _dx; }
		const Vector3<int> voxel_num() const { return _nvoxel; }

		// TODO: HW1
		// part 1.2.
		// Fill the _voxels array with the correct flag.
		void BasicVoxelization() {
			const int nx = _nvoxel[0], ny = _nvoxel[1], nz = _nvoxel[2]; // number of voxels in each dimension
			for (int i = 0; i < nx; ++i)
				for (int j = 0; j < ny; ++j)
					for (int k = 0; k < nz; ++k)
						_voxels[i][j][k] = false;

			// Step 1: Clear _hits.

			// Step 2: shoot rays.

			// Step 3: fill the _voxels array

		}

		// TODO: HW1
		// part 2.1.
		// Fill the _voxels array with the correct flag.
		void AdvancedVoxelization() {
			const int nx = _nvoxel[0], ny = _nvoxel[1], nz = _nvoxel[2]; // number of voxels in each dimension
			for (int i = 0; i < nx; ++i)
				for (int j = 0; j < ny; ++j)
					for (int k = 0; k < nz; ++k)
						_voxels[i][j][k] = false;

			// Step 1: Clear _voxels

			// Step 2: fill the _voxels array
		}

		// TODO: HW1
		// part 3.1.
		// Fill the _voxels array with the correct flag.
		void AdvancedVoxelizationWithApproximation() {
			const int nx = _nvoxel[0], ny = _nvoxel[1], nz = _nvoxel[2]; // number of voxels in each dimension
			for (int i = 0; i < nx; ++i)
				for (int j = 0; j < ny; ++j)
					for (int k = 0; k < nz; ++k)
						_voxels[i][j][k] = false;

			// Step 2: randomly generate a direction and compute inside/outside

			// Step 3: fill the _voxels array
		}

		void WriteVoxelToMesh(const std::string& stl_file_name) const {
			const int nx = _nvoxel[0], ny = _nvoxel[1], nz = _nvoxel[2];
			std::vector<std::vector<Vector3<int>>> faces;
			std::vector<Vector3<int>> corners({
				Vector3<int>(0, 0, 0),
				Vector3<int>(0, 0, 1),
				Vector3<int>(0, 1, 0),
				Vector3<int>(0, 1, 1),
				Vector3<int>(1, 0, 0),
				Vector3<int>(1, 0, 1),
				Vector3<int>(1, 1, 0),
				Vector3<int>(1, 1, 1)
				});
			for (int i = 0; i < nx; ++i)
				for (int j = 0; j < ny; ++j)
					for (int k = 0; k < nz; ++k) {
						if (!_voxels[i][j][k]) continue;
						// Check -x direction.
						Vector3<int> cmin(i, j, k);
						if (i == 0 || !_voxels[i - 1][j][k]) {
							faces.push_back({ cmin + corners[0], cmin + corners[1], cmin + corners[3] });
							faces.push_back({ cmin + corners[0], cmin + corners[3], cmin + corners[2] });
						}
						if (i == nx - 1 || !_voxels[i + 1][j][k]) {
							faces.push_back({ cmin + corners[4], cmin + corners[6], cmin + corners[7] });
							faces.push_back({ cmin + corners[4], cmin + corners[7], cmin + corners[5] });
						}
						if (j == 0 || !_voxels[i][j - 1][k]) {
							faces.push_back({ cmin + corners[0], cmin + corners[4], cmin + corners[5] });
							faces.push_back({ cmin + corners[0], cmin + corners[5], cmin + corners[1] });
						}
						if (j == ny - 1 || !_voxels[i][j + 1][k]) {
							faces.push_back({ cmin + corners[2], cmin + corners[3], cmin + corners[7] });
							faces.push_back({ cmin + corners[2], cmin + corners[7], cmin + corners[6] });
						}
						if (k == 0 || !_voxels[i][j][k - 1]) {
							faces.push_back({ cmin + corners[0], cmin + corners[2], cmin + corners[6] });
							faces.push_back({ cmin + corners[0], cmin + corners[6], cmin + corners[4] });
						}
						if (k == nz - 1 || !_voxels[i][j][k + 1]) {
							faces.push_back({ cmin + corners[5], cmin + corners[7], cmin + corners[3] });
							faces.push_back({ cmin + corners[5], cmin + corners[3], cmin + corners[1] });
						}
					}
			std::ofstream fout(stl_file_name);
			fout << "solid vcg" << std::endl;
			for (const auto& f : faces) {
				std::vector<Vector3<T>> p;
				for (const auto& fi : f) {
					Vector3<T> v = _pmin + fi.cast<T>() * _dx;
					p.push_back(v);
				}
				const Vector3<T> n = (p[1] - p[0]).cross(p[2] - p[1]).normalized();
				fout << "  facet normal " << n.x() << " " << n.y() << " " << n.z() << std::endl;
				fout << "    outer loop" << std::endl;
				for (const auto& v : p) {
					fout << "      vertex " << v.x() << " " << v.y() << " " << v.z() << std::endl;
				}
				fout << "    endloop" << std::endl;
				fout << "  endfacet" << std::endl;
			}
			fout << "endsolid vcg" << std::endl;
		}

		void WriteVoxelToFile(const std::string &voxel_file) const {
			// File format: _pmin.x _pmin.y _pmin.z dx nx ny nz
			// Then a [nx+1][ny+1][nz+1] array of 0s and 1s.
			const int nx = _nvoxel.x(), ny = _nvoxel.y(), nz = _nvoxel.z();


			// Write it to file.
			std::ofstream fout(voxel_file);
			fout << _pmin.x() << " " << _pmin.y() << " " << _pmin.z() << " " << _dx
				<< " " << nx << " " << ny << " " << nz << std::endl;

			for (int i = 0; i < nx; ++i) {
				for (int j = 0; j < ny; ++j) {
					for (int k = 0; k < nz; ++k) {
						if (!_voxels[i][j][k]) {
							fout << "0";
						}
						else {
							fout << "1";
						}
					}
					fout << "0" << std::endl;
				}
				fout << std::string(nz + 1, '0') << std::endl;
			}
			for (int j = 0; j < ny + 1; ++j) {
				fout << std::string(nz + 1, '0') << std::endl;
			}
		}

		using Vector8i = Eigen::Matrix<int, 8, 1>;

		const materials::HexahedralMesh<T> ConvertToHexMesh(std::vector<int>& force, std::vector<int>& fixed, double& num_voxels) {
			std::vector<Vector3<T>> vertices;
			std::vector<Vector8i> elements;
			std::map<int, int> idx_map;
			idx_map.clear();
			// number of possible vertices, not voxels !!
			const int nx = _nvoxel[0] + 1, ny = _nvoxel[1] + 1, nz = _nvoxel[2] + 1;
			int count = 0;
			num_voxels = 0;

			printf("nx: %d, ny: %d, nz: %d\n", nx - 1, ny - 1, nz - 1);

			std::vector<Vector3<int>> corners({
				Vector3<int>(0, 0, 0),
				Vector3<int>(0, 0, 1),
				Vector3<int>(0, 1, 0),
				Vector3<int>(0, 1, 1),
				Vector3<int>(1, 0, 0),
				Vector3<int>(1, 0, 1),
				Vector3<int>(1, 1, 0),
				Vector3<int>(1, 1, 1)
				});

			// loop through number of voxels 
			for (int i = 0; i < nx - 1; ++i)
				for (int j = 0; j < ny - 1; ++j)
					for (int k = 0; k < nz - 1; ++k) {
						if (!_voxels[i][j][k]) continue;
						num_voxels++;

						Vector8i element;
						// push eight vertices to the vertices and elements
						for (int l = 0; l < 8; ++l) {
							int idx = (i + corners[l][0]) * ny * nz + (j + corners[l][1])
								* nz + (k + corners[l][2]);

							if (idx_map.find(idx) == idx_map.end()) {
								idx_map[idx] = count;
								count += 1;
								element(l) = idx_map[idx];
								Vector3<int> vIndex(i + corners[l][0], j + corners[l][1], k + corners[l][2]);
								vertices.push_back(_pmin + vIndex.cast<T>() * _dx);
							}
							else {
								element(l) = idx_map[idx];
							}
						}
						elements.push_back(element);
					}

			// create vertice and element matrix
			materials::Matrix3X<T> vMatrix = MatrixX<T>::Zero(3, vertices.size());
			materials::Matrix8Xi<T> eMatrix = MatrixX<int>::Zero(8, elements.size());

			for (int i = 0; i < vertices.size(); ++i) {
				vMatrix.col(i) = vertices[i];
			}

			for (int i = 0; i < elements.size(); ++i) {
				eMatrix.col(i) = elements[i];
			}

			// find top-z vertices
			for (int i = 0; i < nx - 1; ++i)
				for (int j = 0; j < ny - 1; ++j)
					for (int k = nz - 2; k >= 0; --k) {
						if (!_voxels[i][j][k]) continue;
						printf("i: %d, j: %d, k: %d\n", i, j, k);

						// only store the top points
						for (int l = 1; l < 8; l = l + 2) {
							int idx = (i + corners[l][0]) * ny * nz + (j + corners[l][1])
								* nz + (k + corners[l][2]);
							force.push_back(idx_map[idx]);
						}
						break;
					}
			for (auto i : force) {
				std::cout << i << std::endl;
			}
			std::set<int> forceSet(force.begin(), force.end());
			force.assign(forceSet.begin(), forceSet.end());
			printf("force size: %d\n", force.size());

			std::cout << "top-z done" << std::endl;

			// find left-most vertices
			for (int j = 0; j < ny - 1; ++j)
				for (int k = 0; k < nz - 1; ++k)
					for (int i = 0; i < nx - 1; ++i) {
						if (!_voxels[i][j][k]) continue;

						printf("i: %d, j: %d, k: %d\n", i, j, k);

						// only store the top points
						for (int l = 0; l < 4; l++) {
							int idx = (i + corners[l][0]) * ny * nz + (j + corners[l][1])
								* nz + (k + corners[l][2]);
							fixed.push_back(idx_map[idx]);
						}
						break;
					}
			std::cout << "left-most done" << std::endl;

			// find right-most vertices
			for (int j = 0; j < ny - 1; ++j)
				for (int k = 0; k < nz - 1; ++k)
					for (int i = nx - 2; i >= 0; --i) {
						if (!_voxels[i][j][k]) continue;

						// only store the top points
						for (int l = 4; l < 8; l++) {
							int idx = (i + corners[l][0]) * ny * nz + (j + corners[l][1])
								* nz + (k + corners[l][2]);
							fixed.push_back(idx_map[idx]);
						}
						break;
					}
			std::cout << "right-most done" << std::endl;

			std::set<int> fixedSet(fixed.begin(), fixed.end());
			fixed.assign(fixedSet.begin(), fixedSet.end());
			printf("fixed size: %d\n", fixed.size());

			return materials::HexahedralMesh<T>(vMatrix, eMatrix);
		}

	private:
		std::vector<std::vector<Vector3<T>>> _triangles;
		T _dx;  // The size of each voxel.
		Vector3<T> _pmin;    // The min and max corner of the bounding box.
		Eigen::Vector3i _nvoxel;   // The number of voxels along each direction.
		std::vector<std::vector<std::vector<bool>>> _voxels;   // True <-> voxel is occupied.
	};

}
