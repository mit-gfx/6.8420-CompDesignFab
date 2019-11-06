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

/* ------------------- *
 * Begin Solution Code *
 * ------------------- */
#include <set>
#include "hexahedral_mesh.hpp"
 /* ----------------- *
  * End Solution Code *
  * ----------------- */

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

		void BasicVoxelization() {
			// TODO: HW2
			/* Assignment 2, Part 2.1. */
			/* Implement your code here. */
			// Fill the _voxels array with the correct flag.


			// Step 1: Clear _voxels and _hits.
			const int nx = _nvoxel[0], ny = _nvoxel[1], nz = _nvoxel[2];
			for (int i = 0; i < nx; ++i)
				for (int j = 0; j < ny; ++j)
					for (int k = 0; k < nz; ++k)
						_voxels[i][j][k] = false;

			// set up hits structure
			std::vector<std::vector<std::vector<T>>> hits(nx, std::vector<std::vector<T>>(ny, std::vector<T>()));
			for (int i = 0; i < nx; ++i)
				for (int j = 0; j < ny; ++j)
					hits[i][j].clear();

			/* ------------------- *
			 * Begin Solution Code *
			 * ------------------- */

			 // Step 2: shoot rays.
			int max_cnt = -1;
			for (int i = 0; i < nx; ++i) {
				std::cout << "i = " << i << std::endl;
				for (int j = 0; j < ny; ++j) {
					const Vector3<T> origin = _pmin + Vector3<T>(i + 0.5, j + 0.5, 0.0) * _dx;
					Vector3<T> noise(0, 0, 0);
					int cnt = 0;
					do {
						++cnt;
						ComputeHitsAlongRay(origin + noise, hits[i][j]);
						noise = Vector3<T>::Random() * _dx * 0.1;
						noise.z() = 0.0;
					} while (hits[i][j].size() % 2);
					if (cnt > max_cnt) max_cnt = cnt;
				}
			}
			std::cout << "Max cnt = " << max_cnt << std::endl;
			for (int i = 0; i < nx; ++i) {
				for (int j = 0; j < ny; ++j) {
					auto h = hits[i][j];
					std::sort(h.begin(), h.end());
					const int hit_cnt = static_cast<int>(h.size());
					for (int k = 0; k < hit_cnt; k += 2) {
						const T t0 = h[k];
						const T t1 = h[k + 1];
						const int i0 = FloatToIdx(t0), i1 = FloatToIdx(t1);
						for (int s = i0 - 1; s <= i1 + 1; ++s) {
							if (s < 0 || s >= nz || (s + 0.5) * _dx < t0 || (s + 0.5) * _dx > t1) continue;
							_voxels[i][j][s] = true;
						}
					}
				}
			}
			/* ----------------- *
			 * End Solution Code *
			 * ----------------- */
		}

		void AdvancedVoxelization() {
			// TODO: HW2
			/* Assignment 2, Part 2.2. */
			/* Implement your code here. */
			// Fill the _voxels array with the correct flag.
			const int nx = _nvoxel[0], ny = _nvoxel[1], nz = _nvoxel[2];
			for (int i = 0; i < nx; ++i)
				for (int j = 0; j < ny; ++j)
					for (int k = 0; k < nz; ++k)
						_voxels[i][j][k] = false;

			/* ------------------- *
			 * Begin Solution Code *
			 * ------------------- */
			Vector3<T> new_pmin;
			Vector3<int> new_nvoxel;
			std::vector<std::vector<std::vector<bool>>> new_voxels;
			VoxelizeAlongGeneralNormal(Vector3<T>::UnitZ(), new_pmin, new_nvoxel, new_voxels);
			for (int ii = 0; ii < nx; ++ii) {
				for (int jj = 0; jj < ny; ++jj) {
					for (int kk = 0; kk < nz; ++kk) {
						const Vector3<T> p = _pmin + Vector3<T>(ii + 0.5, jj + 0.5, kk + 0.5) * _dx;
						if (Query(p, Vector3<T>::UnitZ(), new_pmin, new_nvoxel, new_voxels)) _voxels[ii][jj][kk] = true;
					}
				}
			}
			/* ----------------- *
			 * End Solution Code *
			 * ----------------- */
		}

		/* ------------------- *
		 * Begin Solution Code *
		 * ------------------- */
		void VoxelizeAlongGeneralNormal(const Vector3<T>& normal, Vector3<T>& new_pmin, Vector3<int>& new_nvoxel,
			std::vector<std::vector<std::vector<bool>>>& new_voxels) {
			const Eigen::Quaternion<T> q = Eigen::Quaternion<T>::FromTwoVectors(normal, Vector3<T>::UnitZ());
			const auto R = q.matrix();
			auto new_triangles = _triangles;
			for (auto& tri : new_triangles)
				for (auto& v : tri)
					v = R * v;
			new_pmin = new_triangles[0][0];
			Vector3<T> new_pmax = new_triangles[0][0];
			for (const auto& tri : new_triangles)
				for (const auto& v : tri) {
					new_pmin = new_pmin.cwiseMin(v);
					new_pmax = new_pmax.cwiseMax(v);
				}
			new_pmin -= Vector3<T>(1, 1, 1) * _dx;
			new_pmax += Vector3<T>(1, 1, 1) * _dx;
			for (int j = 0; j < 3; ++j)
				new_nvoxel[j] = static_cast<int>((new_pmax[j] - new_pmin[j]) / _dx) + 1;

			std::vector<std::vector<std::vector<T>>> new_hits(new_nvoxel.x(),
				std::vector<std::vector<T>>(new_nvoxel.y()));
			for (const auto& tri : new_triangles) {
				const geometry::Triangle<T> triangle(tri[0], tri[1], tri[2]);
				Vector3<T> vmin = tri[0], vmax = tri[0];
				for (const auto& v : tri) {
					vmin = vmin.cwiseMin(v);
					vmax = vmax.cwiseMax(v);
				}
				vmin -= new_pmin;
				vmax -= new_pmin;
				const int xmin = FloatToIdx(vmin.x()), xmax = FloatToIdx(vmax.x());
				const int ymin = FloatToIdx(vmin.y()), ymax = FloatToIdx(vmax.y());
				for (int i = xmin; i <= xmax; ++i)
					for (int j = ymin; j <= ymax; ++j) {
						const Vector3<T> origin = new_pmin + Vector3<T>(i + 0.5, j + 0.5, 0.0) * _dx;
						const Vector3<T> dir(0, 0, 1);
						const T t = triangle.IntersectRay(origin, dir);
						if (t > -0.5) {
							new_hits[i][j].push_back(t);
						}
					}
			}
			// Convert new_hits to new_voxels.
			new_voxels.resize(new_nvoxel[0]);
			for (int i = 0; i < new_nvoxel[0]; ++i) {
				new_voxels[i].resize(new_nvoxel[1]);
				for (int j = 0; j < new_nvoxel[1]; ++j) {
					new_voxels[i][j] = std::vector<bool>(new_nvoxel[2], false);
					std::vector<T> hits(new_hits[i][j]);
					std::sort(hits.begin(), hits.end());
					const T hmax = -1;
					auto new_hits = hits;
					new_hits.clear();
					for (const auto t : hits) {
						if (new_hits.empty() || t - new_hits.back() > 1e-6) new_hits.push_back(t);
					}
					hits.swap(new_hits);
					if (hits.size() % 2 == 1) hits.push_back(hmax);
					const int hit_cnt = static_cast<int>(hits.size());
					for (int k = 0; k < hit_cnt; k += 2) {
						const T t0 = hits[k];
						const T t1 = hits[k + 1];
						const int i0 = FloatToIdx(t0), i1 = t1 == hmax ? (new_nvoxel[2] - 1) : FloatToIdx(t1);
						for (int s = i0 - 1; s <= i1 + 1; ++s) {
							if (s >= new_nvoxel[2]) break;
							if (s < 0 || (s + 0.5) * _dx < t0 || (s + 0.5) * _dx > t1) continue;
							new_voxels[i][j][s] = true;
						}
					}
				}
			}
		}

		const bool Query(const Vector3<T>& point, const Vector3<T>& normal, const Vector3<T>& new_pmin,
			const Vector3<int>& new_nvoxel, const std::vector<std::vector<std::vector<bool>>>& new_voxels) {
			const Eigen::Quaternion<T> q = Eigen::Quaternion<T>::FromTwoVectors(normal, Vector3<T>::UnitZ());
			const auto R = q.matrix();
			const Vector3<T> new_point = R * point;
			const Vector3<T> new_pmax = new_pmin + new_nvoxel.cast<T>() * _dx;
			if ((new_point - new_pmin).minCoeff() <= 0.0 || (new_point - new_pmax).maxCoeff() >= 0.0) return false;
			Vector3<int> idx;
			for (int i = 0; i < 3; ++i) idx[i] = int((new_point[i] - new_pmin[i]) / _dx);
			// This should never happen but just in case...
			for (int i = 0; i < 3; ++i) {
				if (idx[i] < 0) idx[i] == 0;
				if (idx[i] > new_nvoxel[i] - 1) idx[i] = new_nvoxel[i] - 1;
			}
			return new_voxels[idx[0]][idx[1]][idx[2]];
		}
		/* ----------------- *
		 * End Solution Code *
		 * ----------------- */

		void AdvancedVoxelizationWithApproximation() {
			// TODO: HW2
			/* Assignment 2, Part 2.3. */
			/* Implement your code here. */
			// Fill the _voxels array with the correct flag.
			const int nx = _nvoxel[0], ny = _nvoxel[1], nz = _nvoxel[2];
			for (int i = 0; i < nx; ++i)
				for (int j = 0; j < ny; ++j)
					for (int k = 0; k < nz; ++k)
						_voxels[i][j][k] = false;

			/* ------------------- *
			 * Begin Solution Code *
			 * ------------------- */
			 // Step 2: randomly generate a plane.
			const int max_trial = 11;
			std::vector<std::vector<std::vector<int>>> count(nx, std::vector<std::vector<int>>(ny, std::vector<int>(nz, 0)));
			for (int i = 0; i < max_trial; ++i) {
				const Vector3<T> normal = Vector3<T>::Random().normalized();
				Vector3<T> new_pmin;
				Vector3<int> new_nvoxel;
				std::vector<std::vector<std::vector<bool>>> new_voxels;
				VoxelizeAlongGeneralNormal(normal, new_pmin, new_nvoxel, new_voxels);
				for (int ii = 0; ii < nx; ++ii)
					for (int jj = 0; jj < ny; ++jj) {
						for (int kk = 0; kk < nz; ++kk) {
							const Vector3<T> p = _pmin + Vector3<T>(ii + 0.5, jj + 0.5, kk + 0.5) * _dx;
							if (Query(p, normal, new_pmin, new_nvoxel, new_voxels)) count[ii][jj][kk]++;
						}
					}
			}
			for (int i = 0; i < nx; ++i) {
				for (int j = 0; j < ny; ++j) {
					for (int k = 0; k < nz; ++k) {
						if (count[i][j][k] * 2 > max_trial) _voxels[i][j][k] = true;
					}
				}
			}
			/* ----------------- *
			 * End Solution Code *
			 * ----------------- */
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

	private:

		/* ------------------- *
		 * Begin Solution Code *
		 * ------------------- */

		 // We assume the origin must be strictly outside the bounding box.
		void ComputeHitsAlongRay(const Vector3<T>& origin, std::vector<T>& hits) const {
			hits.clear();
			const Vector3<T> dir(0, 0, 1);
			for (const auto& vec : _triangles) {
				geometry::Triangle<T> triangle(vec[0], vec[1], vec[2]);
				const T new_t = triangle.IntersectRay(origin, dir);
				if (new_t > -0.5) hits.push_back(new_t);
			}
		}
		const int FloatToIdx(const T t) const {
			return static_cast<int>(t / _dx);
		}

		/* ----------------- *
		 * End Solution Code *
		 * ----------------- */

		std::vector<std::vector<Vector3<T>>> _triangles;
		T _dx;  // The size of each voxel.
		Vector3<T> _pmin;    // The min and max corner of the bounding box.
		Eigen::Vector3i _nvoxel;   // The number of voxels along each direction.
		std::vector<std::vector<std::vector<bool>>> _voxels;   // True <-> voxel is occupied.
	};

}
