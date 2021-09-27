#pragma once
#include <Eigen/Dense>
#include <vector>

namespace geometry {
    template <typename T>
    using Vector3 = Eigen::Matrix<T, 3, 1>;

    // the plane is represented by (x - _p) /dot _normal = 0
    template <typename T>
    class Plane {
    public:
        Plane(Vector3<T> p, Vector3<T> normal) {
            _p = p;
            _normal = normal;
            _normal.normalize();
        }

        Vector3<T>& p() { return _p; }
        Vector3<T>& normal() { return _normal; }
        
        // return if the point is on plane
        // also fill parameter dist as the signed distance from point to plane
        bool onPlane(Vector3<T> point, T& dist) {
            dist = (point - _p).dot(_normal);
            if (std::fabs(dist) < 1e-6) {
                return true;
            } else {
                return false;
            }
        }

    private:
        Vector3<T> _p;
        Vector3<T> _normal;
    };

    template <typename T>
    class Triangle {
    public:
        Triangle(Vector3<T> v0, Vector3<T> v1, Vector3<T> v2) {
            _vertices[0] = v0;
            _vertices[1] = v1;
            _vertices[2] = v2;
        }

        Vector3<T>* vertices() { return _vertices; }
        Vector3<T>& vertices(int idx) { return _vertices[idx]; }

        
        // TODO: HW1
        // part 2.1
        // Implement the function to do intersection between triangle and plane p
        // Input: plane p
        // Output: intersections points with three edges
        // Hint:
        //      - enumerate three edges of the triangle and do intersection individually
        //      - consider the case that no intersection
        //      - consider how to avoid repeated intersection points in returned list
        std::vector<Vector3<T>> IntersectPlane(Plane<T> p) {
            std::vector<Vector3<T>> intersections;
            intersections.clear();

            // identify how many points are on the plane and the sign of the signed distance
            // if 3 points are on the plane, the entire triangle is on the plane
            // in this case, return all 3 points
            // if 2 points are on the plane, one side of the triangle is on the plane
            // in this case, return the 2 points that are on the plane
            // if 1 point is on the plane, return that point and (potentially) the intersection on the other edge
            // for each edge, there will be an intersection if and only if the signed distance of each vertex from the plane has different signs
            // if the signs are different, use proportion to determine the intersection point
            // in particular, if vertex1 has dist d1 and vertex2 has dist d2 from the plane where d1 * d2 < 0
            // then the intersection point is v1 + (v2 - v1) * d1 / (d1 + d2)

            T dist[3];
            bool isOnPlane[3];
            int onPlaneCounter = 0;
            for (int i = 0 ; i < 3 ; i++) {
                isOnPlane[i] = p.onPlane(_vertices[i], dist[i]);
                if (isOnPlane[i]) {
                    onPlaneCounter++;
                }
            }

            if (onPlaneCounter == 0) {
                for (int i = 0 ; i < 3 ; i++) {
                    Vector3<T> v1 = _vertices[i], v2;
                    T d1 = dist[i], d2;
                    if (i == 2) {
                        v2 = _vertices[0];
                        d2 = dist[0];
                    } else {
                        v2 = _vertices[i+1];
                        d2 = dist[i+1];
                    }

                    if (d1 * d2 < 0) {
                        Vector3<T> intersectionPoint = v1 + (v2 - v1) * d1 / (d1 + d2);
                        intersections.push_back(intersectionPoint);
                    }
                }
            } else if (onPlaneCounter == 1) {
                Vector3<T> v1, v2;
                T d1, d2;
                if (isOnPlane[0]) {
                    intersections.push_back(_vertices[0]);
                    v1 = _vertices[1];
                    v2 = _vertices[2];
                    d1 = dist[1];
                    d2 = dist[2];
                } else if (isOnPlane[1]) {
                    intersections.push_back(_vertices[1]);
                    v1 = _vertices[0];
                    v2 = _vertices[2];
                    d1 = dist[0];
                    d2 = dist[2];
                } else {
                    intersections.push_back(_vertices[2]);
                    v1 = _vertices[1];
                    v2 = _vertices[0];
                    d1 = dist[1];
                    d2 = dist[0];
                }
            } else if (onPlaneCounter == 2 || onPlaneCounter == 3) {
                for (int i = 0 ; i < 3 ; i++) {
                    if (isOnPlane[i]) {
                        intersections.push_back(_vertices[i]);
                    }
                }
            }
            return intersections;
        }

        // TODO: HW2
        // part 1.1
        // Implement the function to do intersection between triangle and a ray
        // Input: a ray, the ray is represented by an origin position and a direction vector
        // Output: return a real number t, the intersection is origin + dir * t, t = -1 means no intersection
        const T IntersectRay(const Vector3<T>& origin, const Vector3<T>& dir) const {
            const T flag = static_cast<T>(-1.0);    
            return flag;
        }

    private:
        Vector3<T> _vertices[3];
    };
}