#pragma once
#include "mesh.hpp"
#include <string>
#include <cstring>
#include <vector>

namespace mesh {
    template <typename T>
    bool ReadSTL(std::string file_name,
        std::vector<std::vector<Vector3<T>>>& triangles,
        std::vector<Vector3<T>>& normals) {
        
        FILE* fp = std::fopen(file_name.c_str(), "r");
        
        if (fp == NULL) {
            printf("No STL file found\n");
            return false;
        }

        triangles.clear();
        normals.clear();

        char input[80];
        for (;;) {
            fscanf(fp, "%s", input);
            if (input == std::string("endsolid")) {
                // reach end of file
                break;
            }
            for (;input != std::string("facet");) {
                fscanf(fp, "%s", input);
            }

            std::vector<Vector3<T>> triangle;
            Vector3<T> normal;
            if (std::is_same<T, float>::value) {
                float nx, ny, nz;
                fscanf(fp, "%s %f %f %f\n", input, &nx, &ny, &nz);
                normal[0] = nx; normal[1] = ny; normal[2] = nz;
            }
            else 
                fscanf(fp, "%s %lf %lf %lf\n", input, &normal[0], &normal[1], &normal[2]);

            fscanf(fp, "%s %s", input, input);

            triangle.clear();
            for (int i = 0;i < 3;++i) {
                Vector3<T> p;
                if (std::is_same<T, float>::value) {
                    float px, py, pz;
                    fscanf(fp, "%s %f %f %f\n", input, &px, &py, &pz);
                    p[0] = px; p[1] = py; p[2] = pz;
                }
                else
                    fscanf(fp, "%s %lf %lf %lf\n", input, &p[0], &p[1], &p[2]);
                triangle.push_back(p);
            }
            fscanf(fp, "%s %s", input, input);
            
            triangles.push_back(triangle);
            normals.push_back(normal);
        }

        fclose(fp);
        return true;
    }
}
