#pragma once
#include "mesh.hpp"
#include "read_stl.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <map>


namespace mesh {
    template<typename T>
    static bool Vertex_Cmp(Vector3<T> A, Vector3<T> B) {
        if (A(0) < B(0) - 1e-7) return true;
        if (A(0) > B(0) + 1e-7) return false;
        if (A(1) < B(1) - 1e-7) return true;
        if (A(1) > B(1) + 1e-7) return false;
        if (A(2) < B(2) - 1e-7) return true;
        return false;
    }

    template<typename T>
    class TriMesh {
    
    public:
        TriMesh() {
            _vertices.clear();
            _elements.clear();
            _normals.clear();
        }

        /* 
            input: a file name for stl model, and a scale value.
            output: a connected triangle mesh with scaled vertex coordinates 
        */
        TriMesh(std::string stl_file_name, T scale = 1.0) {
            std::vector<std::vector<Vector3<T>>> triangles;
            std::vector<Vector3<T>> normals;
            if (!ReadSTL(stl_file_name, triangles, normals))
                return;

            for (int i = 0;i < triangles.size();++i) {
                for (int j = 0;j < 3;++j) {
                    triangles[i][j] *= scale;
                }
            }
            
            // collect all vertices from triangle soup
            std::vector<Vector3<T>> vertex_list;
            vertex_list.clear();
            for (int i = 0;i < triangles.size();++i)
                for (int j = 0;j < 3;++j)
                    vertex_list.push_back(triangles[i][j]);
            std::sort(vertex_list.begin(), vertex_list.end(), 
                [](const Vector3<T>& A, const Vector3<T>& B) -> bool {
                    if (A(0) < B(0) - 1e-7) return true;
                    if (A(0) > B(0) + 1e-7) return false;
                    if (A(1) < B(1) - 1e-7) return true;
                    if (A(1) > B(1) + 1e-7) return false;
                    if (A(2) < B(2) - 1e-7) return true;
                    return false;
                });

            // delete duplicated vertices
            _vertices.clear();
            _vertices.push_back(vertex_list[0]);
            for (int i = 1;i < vertex_list.size();++i) {
                if (Vertex_Cmp(vertex_list[i - 1], vertex_list[i])) {
                    _vertices.push_back(vertex_list[i]);
                }
            }

            // construct indices of vertices for each triangle
            _elements.clear();
            for (int i = 0;i < triangles.size();++i) {
                Eigen::Vector3i element(find_index(triangles[i][0]),
                    find_index(triangles[i][1]),
                    find_index(triangles[i][2]));
                _elements.push_back(element);
            }

            _normals = normals;

            /*
                index all edges for each triangle.
                generated _edges[i][j] represents the index of the jth edge in ith triangle.
            */
            CreateEdges();
        }

        // write mesh to obj file
        void WriteToObj(std::string obj_file_name) {
            FILE* fp = fopen(obj_file_name.c_str(), "w");
            for (int i = 0;i < _vertices.size();++i)
                if (std::is_same<T, float>::value)
                    fprintf(fp, "v %.6f %.6f %.6f\n", _vertices[i][0], _vertices[i][1], _vertices[i][2]);
                else
                    fprintf(fp, "v %.6lf %.6lf %.6lf\n", _vertices[i][0], _vertices[i][1], _vertices[i][2]);
            for (int i = 0;i < _elements.size();++i)
                fprintf(fp, "f %d %d %d\n", _elements[i][0] + 1, _elements[i][1] + 1, _elements[i][2] + 1);
            fclose(fp);
        }

		// Transform object with uniform scaling and followed by translation
		void Transform(Vector3<T> translation, T scale) {
			for (auto& v : _vertices) {
				v *= scale;
				v += translation;
			}
		}

        // get functions for variables
        std::vector<Vector3<T>>& vertices() { return _vertices; }
        Vector3<T>& vertices(int idx) { return _vertices[idx]; }
        std::vector<Eigen::Vector3i>& elements() { return _elements; }
        Eigen::Vector3i& elements(int idx) { return _elements[idx]; }
        std::vector<Vector3<T>>& normals() { return _normals; }
        Vector3<T>& normals(int idx) { return _normals[idx]; }
        std::vector<Eigen::Vector3i>& edges() { return _edges; }
        Eigen::Vector3i& edges(int idx) { return _edges[idx]; }
        int num_vertices() { return _vertices.size(); }
        int num_edges() { return _num_edges; }

    private:
        void CreateEdges() {
            std::map<std::pair<int, int>, int> _edge_index_mapping;
            _num_edges = 0;
            _edge_index_mapping.clear();
            _edges = _elements;                 // _edges has same size as _elements
            for (int i = 0;i < _elements.size();++i)
                for (int j = 0;j < 3;++j) {
                    int idx0 = _elements[i][j];
                    int idx1 = _elements[i][(j + 1) % 3];
                    if (idx0 > idx1)
                        std::swap(idx0, idx1);
                    std::pair<int, int> edge = std::make_pair(idx0, idx1);
                    auto it = _edge_index_mapping.find(edge);
                    if (it == _edge_index_mapping.end()) {
                        _edge_index_mapping[edge] = _num_edges;
                        _edges[i][j] = _num_edges ++;
                    } else {
                        _edges[i][j] = it->second;
                    }
                }
        }

        int find_index(Vector3<T> p) {
            int l = 0, r = _vertices.size() - 1;
            for (;l < r;) {
                int mid = (l + r) >> 1;
                if (Vertex_Cmp(_vertices[mid], p))
                    l = mid + 1;
                else
                    r = mid;
            }
            return l;
        }

        std::vector<Vector3<T>> _vertices;                      // list of vertices
        std::vector<Eigen::Vector3i> _elements;                 // list of elements (triangles) - each element stores the indices of three vertices in a triangle
        std::vector<Eigen::Vector3i> _edges;                    // list of triangle edges - each triplet stores the indices of three edges in a triangle (different from vertex indices)
        std::vector<Vector3<T>> _normals;                       // list of normal for each triangle
        int _num_edges;                                         // number of different edges
    };
}