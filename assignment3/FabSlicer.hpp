#pragma once
#include "tri_mesh.hpp"
#include "BasicGeometry.hpp"
#include "IntervalTree.hpp"
#include "GCodeConverter.hpp"
#include <chrono>

namespace fab_translation {
    template <typename T>
    using Vector3 = Eigen::Matrix<T, 3, 1>;

    // Intersection Edge class defines an edge from slicing algorithm.
    // _p0 and _p1 are the coordinates of its two end point, please do not change their names or delete them from this class
    // you can add any variable/function in this class for your convenience.
    template <typename T>
    class IntersectionEdge {
    public:
        IntersectionEdge(int id0, int id1, Vector3<T> p0, Vector3<T> p1)
            : _id0(id0), _id1(id1), _p0(p0), _p1(p1) {
                if (_id0 > _id1) {
                    std::swap(_id0, _id1);
                    std::swap(_p0, _p1);
                }
            }
        const int id0() const { return _id0; }
        const int id1() const { return _id1; }
        Vector3<T> p0() { return _p0; }
        Vector3<T> p1() { return _p1; }

    private:
        int _id0, _id1;
        Vector3<T> _p0, _p1;
    };

    template <typename T>
    class FabSlicer {
        
    public:
        // TODO: HW3
        // part 3: you may want to modify the initialization code to help implement the accelerated algorithms.
        //         e.g. bulid the interval tree
        /*
            params:
                tri_mesh: the triangle mesh needed to be sliced.
                n_layers: the number of slicing layers.
                n_infill_resolution: a number indicate the resolution of your infill pattern, 
                                     the value can be interpreted by yourself according to your infill pattern.
        */
        FabSlicer(mesh::TriMesh<T> tri_mesh,
			Vector3<T> bed_min,
			Vector3<T> bed_max,
			T dz,
			T infill_dx)
            : _tri_mesh(tri_mesh) {
            
            // find the object bounds
            
			Vector3<T> obj_min = tri_mesh.vertices(0);
			Vector3<T> obj_max = tri_mesh.vertices(0);
			
            for (int i = 0;i < tri_mesh.vertices().size();++i) {
				obj_min[0] = std::min(obj_min[0], tri_mesh.vertices(i)[0]);
				obj_max[0] = std::max(obj_max[0], tri_mesh.vertices(i)[0]);
				obj_min[1] = std::min(obj_min[1], tri_mesh.vertices(i)[1]);
				obj_max[1] = std::max(obj_max[1], tri_mesh.vertices(i)[1]);
				obj_min[2] = std::min(obj_min[2], tri_mesh.vertices(i)[2]);
				obj_max[2] = std::max(obj_max[2], tri_mesh.vertices(i)[2]);
            }


			// Scale model to fit in 3d printer bounds if necessary, 
			// and translate to the center of the bed

			Vector3<T> bed_dim = bed_max - bed_min;
			Vector3<T> obj_dim = obj_max - obj_min;

			T scale = 1.0;
			for (int i = 0; i < 3; ++i) {
				if (bed_dim[i] < obj_dim[i]) {
					scale = std::min(scale, bed_dim[i] / obj_dim[i]);
				}
			}

			Vector3<T> obj_bottom_center = (obj_max + obj_min) / 2;
			obj_bottom_center[2] = obj_min[2];
			Vector3<T> bed_center = (bed_max + bed_min) / 2;
			bed_center[2] = bed_min[2];
			Vector3<T> translation = bed_center - obj_bottom_center;
			Vector3<T> scaled_translation = obj_bottom_center * (1.0 - scale) + translation;

			_tri_mesh.Transform(scaled_translation, scale);

			_bottom = obj_min[2] * scale + scaled_translation[2];
			_top = obj_max[2] * scale + scaled_translation[2];
			_dz = dz;
            _infill_dx = infill_dx;

			T z_min = _bottom;
			T z_max = _top;

			int t1 = std::clock();
            _interval_tree = data_structure::IntervalTree<T>(z_min - 0.1, z_max + 0.1);

            std::vector<data_structure::IntervalEntry<T>> triangle_intervals;
            triangle_intervals.clear();
            for (int i = 0;i < _tri_mesh.elements().size();++i) {
                Eigen::Vector3i& element = _tri_mesh.elements(i);
                T l = _tri_mesh.vertices(element[0])[2];
                T r = _tri_mesh.vertices(element[0])[2];
                for (int j = 1;j < 3;++j) {
                    l = std::min(l, _tri_mesh.vertices(element[j])[2]);
                    r = std::max(r, _tri_mesh.vertices(element[j])[2]);
                }
                data_structure::IntervalEntry<T> interval(l - 1e-4, r + 1e-4, i);
                triangle_intervals.push_back(interval);
            }

            _interval_tree.build(0,  _interval_tree.lower_bound(), _interval_tree.upper_bound(), triangle_intervals);
			int t2 = std::clock();
			printf("building interval tree success... %.6lf seconds\n", (double)(t2 - t1) / 1000000.0);
            // ---------------------------------------------------------------------------------------------

            // compute infill x-y range
            _infill_x_lower_bound = 1000000.0; _infill_x_upper_bound = -1000000.0;
            _infill_y_lower_bound = 1000000.0; _infill_y_upper_bound = -1000000.0;
            for (int i = 0;i < _tri_mesh.vertices().size();++i) {
                _infill_x_lower_bound = std::min(_infill_x_lower_bound, _tri_mesh.vertices(i)[0]);
                _infill_x_upper_bound = std::max(_infill_x_upper_bound, _tri_mesh.vertices(i)[0]);
                _infill_y_lower_bound = std::min(_infill_y_lower_bound, _tri_mesh.vertices(i)[1]);
                _infill_y_upper_bound = std::max(_infill_y_upper_bound, _tri_mesh.vertices(i)[1]);
            }

            _infill_x_lower_bound -= _infill_dx * 0.5;
            _infill_x_upper_bound += _infill_dx * 0.5;
            _infill_y_lower_bound -= _infill_dx * 0.5;
            _infill_y_upper_bound += _infill_dx * 0.5;
        }

        /*
            *** Please do not modify this function! ***
            params:
                contour: save the contour generated by your code, 
                         contour[i][j][k] is an Eigen::Vector3 represents the coordinates of kth waypoint on jth contours in ith layer
                infill_edges: save the infill edges of your infill pattern generated by your code
                         infill_edges[i][j] is a pair<Eigen::Vector3, Eigen::Vector3> represents the coordinates of two end point of jth infill edge in ith layer
                bruteforce: whether or not use bruteforce algorithm to do slicing
        */
        void RunTranslation(
            std::vector<std::vector<std::vector<Vector3<T>>>>& contour,
            std::vector<std::vector<std::pair<Vector3<T>, Vector3<T>>>>& infill_edges,
            bool bruteforce = false) {
            
            _tri_mesh.WriteToObj(std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/results/mesh.obj");

            std::vector<std::vector<fab_translation::IntersectionEdge<T>>> intersection_edges;
            
            std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
            if (bruteforce) {
                Slicing_bruteforce(_tri_mesh, intersection_edges);
            } else {
                Slicing_accelerated(_tri_mesh, intersection_edges);
            }
            std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
            double slicing_time = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
            printf("slicing mesh finished in %.6lf seconds\n", slicing_time);

            VisualizeSlicing(std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/results/slicing.ply", 0.5, intersection_edges);

            t_start = std::chrono::high_resolution_clock::now();
            CreateContour(_tri_mesh, intersection_edges, contour);
            t_end = std::chrono::high_resolution_clock::now();
            double contour_time = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
            printf("stitching edges finished in %.6lf seconds\n", contour_time);

            VisualizeContour(std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/results/contour.ply", 0.5, contour);

            t_start = std::chrono::high_resolution_clock::now();
            Infill(contour, infill_edges);
            t_end = std::chrono::high_resolution_clock::now();
            double infill_time = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
            printf("infilling finished in %.6lf seconds\n", infill_time);

            VisualizeInfill(std::string(PROJECT_SOURCE_DIR) + "/data/assignment3/results/infill.ply", 0.5, infill_edges);
        }

        // Input: 
        //      tri_mesh: the triangle mesh needed to be sliced
        // Output:
        //      intersection_edges: soup of intersection edges for each layer
        void Slicing_bruteforce(mesh::TriMesh<T>& tri_mesh, 
            std::vector<std::vector<IntersectionEdge<T>>> &intersection_edges) {
            
            std::vector<Eigen::Vector3i>& elements = tri_mesh.elements();
            std::vector<Vector3<T>>& vertices = tri_mesh.vertices();
            std::vector<Eigen::Vector3i>& edges = tri_mesh.edges();

            intersection_edges.clear();
            for (T h = _bottom; h <= _top; h += _dz) {
                std::vector<IntersectionEdge<T>> intersections_one_plane;
                intersections_one_plane.clear();

                geometry::Plane<T> plane(Vector3<T>(0, 0, h), Vector3<T>(0, 0, 1));
                for (int i = 0;i < elements.size();++i) {
                    geometry::Triangle<T> triangle(vertices[elements[i](0)], vertices[elements[i](1)], vertices[elements[i](2)]);
                    std::vector<std::pair<int, Vector3<T>>> intersections = triangle.IntersectPlane(plane);
                    int num_intersections = 0;
                    for (int j = 0;j < 3;++j) {
                        if ((intersections[j].first) != -1) {
                            ++ num_intersections;
                        }
                    }
                    if (num_intersections == 2) {
                        int cnt = 0;
                        int id[2];
                        Vector3<T> p[2];
                        for (int j = 0;j < 3;++j) {
                            if (intersections[j].first != -1) {
                                p[cnt] = intersections[j].second;
                                if (intersections[j].first != 3) {
                                    id[cnt] = elements[i][j];
                                } else {
                                    id[cnt] = vertices.size() + edges[i][j];
                                }
                                ++ cnt;
                            }
                        }
                        assert(cnt == 2);
                        intersections_one_plane.push_back(IntersectionEdge<T>(id[0], id[1], p[0], p[1]));
                    }
                }

                intersection_edges.push_back(intersections_one_plane);
            }
        }

        // Input: 
        //      tri_mesh: the triangle mesh needed to be sliced
        // Output:
        //      intersection_edges: soup of intersection edges for each layer
        void Slicing_accelerated(mesh::TriMesh<T>& tri_mesh,
            std::vector<std::vector<IntersectionEdge<T>>> &intersection_edges) {
            
            std::vector<Eigen::Vector3i>& elements = tri_mesh.elements();
            std::vector<Vector3<T>>& vertices = tri_mesh.vertices();
            std::vector<Eigen::Vector3i>& edges = tri_mesh.edges();

            intersection_edges.clear();
            for (T h = _bottom; h <= _top; h += _dz) {

                std::vector<data_structure::IntervalEntry<T>> candidates;
                _interval_tree.query(0, _interval_tree.lower_bound(), _interval_tree.upper_bound(), h, candidates);

                std::vector<IntersectionEdge<T>> intersections_one_plane;
                intersections_one_plane.clear();

                geometry::Plane<T> plane(Vector3<T>(0, 0, h), Vector3<T>(0, 0, 1));

                for (int i = 0;i < candidates.size();++i) {
                    int ii = candidates[i].id;
                    geometry::Triangle<T> triangle(vertices[elements[ii](0)], vertices[elements[ii](1)], vertices[elements[ii](2)]);
                    std::vector<std::pair<int, Vector3<T>>> intersections = triangle.IntersectPlane(plane);
                    int num_intersections = 0;
                    for (int j = 0;j < 3;++j) {
                        if ((intersections[j].first) != -1) {
                            ++ num_intersections;
                        }
                    }
                    if (num_intersections == 2) {
                        int cnt = 0;
                        int id[2];
                        Vector3<T> p[2];
                        for (int j = 0;j < 3;++j) {
                            if (intersections[j].first != -1) {
                                p[cnt] = intersections[j].second;
                                if (intersections[j].first != 3) {
                                    id[cnt] = elements[ii][j];
                                } else {
                                    id[cnt] = vertices.size() + edges[ii][j];
                                }
                                ++ cnt;
                            }
                        }
                        assert(cnt == 2);
                        intersections_one_plane.push_back(IntersectionEdge<T>(id[0], id[1], p[0], p[1]));    
                    }
                }

                intersection_edges.push_back(intersections_one_plane);
            }
        }

        // TODO: HW3
        // part 2.1, 5.1
        // Input:
        //      tri_mesh: the triangle mesh needed to be sliced
        //      intersection_edges: the intersection edge soup, which is the output from slicing_bruteforce/slicing_accelerated
        // Output:
        //      contours: the contours in each layer. 
        //                contour[i][j][k] is an Eigen::Vector3 represents the coordinates of kth waypoint on jth contours in ith layer
        // Hints:
        //      1. think of how to connect cutting edges.
        //      2. some edges may be isolated that cannot form a contour, please detect and remove them.
        //      3. there may be more than one contours in one layer, how to detect and handle this case.
        //      3. how to do it fast? (fast nearest neighor searching? indexing edges?) (part 3.2)
        void CreateContour(mesh::TriMesh<T>& tri_mesh,
            std::vector<std::vector<IntersectionEdge<T>>> &intersection_edges,
            std::vector<std::vector<std::vector<Vector3<T>>>>& contours) {

        }

        // TODO: HW3
        // part 4.1
        // Make infill pattern in each layer.
        // Input:
        //      contours: the contours in each layer, which is generated from CreateContour function
        // Output:
        //      infill_edges: the infill edge soup for each layer.
        //                    infill_edges[i][j] is a pair<Eigen::Vector3, Eigen::Vector3> represents 
        //                    the coordinates of two end point of jth infill edge in ith layer
        void Infill(std::vector<std::vector<std::vector<Vector3<T>>>>& contours,
            std::vector<std::vector<std::pair<Vector3<T>, Vector3<T>>>>& infill_edges) {
            
            // ------------------- Delete this part of code before implementation ------------------------
            infill_edges.clear();

            for (int i = 0;i < contours.size();i++) {
                infill_edges.push_back(std::vector<std::pair<Vector3<T>, Vector3<T>>>());
            }
            // -------------------------------------------------------------------------------------------
        }  

        /*
            visualize your slicing results by PLY file (can be opened in MeshLab)
        */
        void VisualizeSlicing(std::string file_name, 
            T point_density,
            std::vector<std::vector<IntersectionEdge<T>>> intersection_edges) {
            
            // generate point cloud for ply
            std::vector<Vector3<T>> points;
            points.clear();
            for (int i = 0;i < intersection_edges.size();++i)
                for (int j = 0;j < intersection_edges[i].size();++j) {
                    Vector3<T> s_pos = intersection_edges[i][j].p0();
                    Vector3<T> t_pos = intersection_edges[i][j].p1();
                    int num_steps = (int)((t_pos - s_pos).norm() / point_density) + 1;
                    for (int step = 0;step <= num_steps;++step) {
                        Vector3<T> pos = t_pos * ((T)step / num_steps) + s_pos * ((T)1.0 - (T)step / num_steps);
                        points.push_back(pos);
                    }
                }

            // output to ply
            FILE* fp = fopen(file_name.c_str(), "w");
            fprintf(fp, "ply\nformat ascii 1.0\n");
            fprintf(fp, "element vertex %d\n", (int)points.size());
            fprintf(fp, "property float32 x\nproperty float32 y\nproperty float32 z\n");
            fprintf(fp, "end_header\n");
            for (int i = 0;i < points.size();++i)
                if (std::is_same<T, float>::value)
                    fprintf(fp, "%.6f %.6f %.6f\n", points[i](0), points[i](1), points[i](2));
                else
                    fprintf(fp, "%.6lf %.6lf %.6lf\n", points[i](0), points[i](1), points[i](2));
            fclose(fp);
        }

        /*
            visualize your contour results by PLY file (can be opened in MeshLab)
        */
        void VisualizeContour(std::string file_name,
            T point_density, 
            std::vector<std::vector<std::vector<Vector3<T>>>>& contour) {
            // generate point cloud for ply
            std::vector<std::vector<Vector3<T>>> points;
            int num_points = 0;
            points.clear();
            for (int i = 0;i < contour.size();++i)
                for (int j = 0;j < contour[i].size();++j) {
                    std::vector<Vector3<T>> one_contour;
                    for (int k = 0;k < contour[i][j].size();++k) {
                        Vector3<T> s_pos = contour[i][j][k];
                        Vector3<T> t_pos = contour[i][j][(k + 1) % contour[i][j].size()];
                        int num_steps = (int)((t_pos - s_pos).norm() / point_density) + 1;
                        for (int step = 0;step < num_steps;++step) {
                            Vector3<T> pos = t_pos * ((T)step / num_steps) + s_pos * ((T)1.0 - (T)step / num_steps);
                            one_contour.push_back(pos);
                            ++ num_points;
                        }
                    }
                    points.push_back(one_contour);
                }

            // output to ply
            FILE* fp = fopen(file_name.c_str(), "w");
            fprintf(fp, "ply\nformat ascii 1.0\n");
            fprintf(fp, "element vertex %d\n", num_points);
            fprintf(fp, "property float32 x\nproperty float32 y\nproperty float32 z\n");
            fprintf(fp, "end_header\n");
            for (int i = 0;i < points.size();++i) {
                for (int j = 0;j < points[i].size();++j) {
                    if (std::is_same<T, float>::value)
                        fprintf(fp, "%.6f %.6f %.6f\n", points[i][j](0), points[i][j](1), points[i][j](2));
                    else
                        fprintf(fp, "%.6lf %.6lf %.6lf\n", points[i][j](0), points[i][j](1), points[i][j](2));
                }
                fprintf(fp, "\n");
            }
            fclose(fp);
        }

        void VisualizeInfill(std::string file_name,
            T point_density,
            std::vector<std::vector<std::pair<Vector3<T>, Vector3<T>>>>& infill_edges) {
            // generate point cloud for ply
            std::vector<Vector3<T>> points;
            points.clear();
            for (int i = 0;i < infill_edges.size();++i)
                for (int j = 0;j < infill_edges[i].size();++j) {
                    int num_steps = (int)((infill_edges[i][j].first - infill_edges[i][j].second).norm() / point_density) + 1;
                    for (int k = 0;k <= num_steps;++k)
                        points.push_back(infill_edges[i][j].first + (infill_edges[i][j].second - infill_edges[i][j].first) * (T)k / (T)num_steps);
                }

            // output to ply
            FILE* fp = fopen(file_name.c_str(), "w");
            fprintf(fp, "ply\nformat ascii 1.0\n");
            fprintf(fp, "element vertex %d\n", (int)points.size());
            fprintf(fp, "property float32 x\nproperty float32 y\nproperty float32 z\n");
            fprintf(fp, "end_header\n");
            for (int i = 0;i < points.size();++i)
                if (std::is_same<T, float>::value)
                    fprintf(fp, "%.6f %.6f %.6f\n", points[i](0), points[i](1), points[i](2));
                else
                    fprintf(fp, "%.6lf %.6lf %.6lf\n", points[i](0), points[i](1), points[i](2));
            fclose(fp);
        }

    private:
        mesh::TriMesh<T> _tri_mesh;

        /* Variables for slicing */
        T _bottom, _top, _dz;

        /* Variables for infill algorithm */
        T _infill_dx; // infill pattern will be equal-length grid
        T _infill_x_lower_bound, _infill_x_upper_bound;
        T _infill_y_lower_bound, _infill_y_upper_bound;

        /* accelerated data structure */
        data_structure::IntervalTree<T> _interval_tree;
    };
}
