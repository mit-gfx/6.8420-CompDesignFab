// Jie Xu
// jiex@csail.mit.edu
// Aug 21, 2019
#pragma once
#include "deformable_body.hpp"
#include "tetrahedral_mesh.hpp"
#include "typedefs.hpp"
#include <iostream>

namespace materials {
    template<typename T>
    class TetDeformableBody : public DeformableBody<3, T> {
    public:
        TetDeformableBody(const Material<3, T>& material,
                            const Matrix3X<T>& initial_vertex_position,
                            const T density, const TetrahedralMesh<T>& undeformed_tet_mesh)
                            : DeformableBody<3, T>(material, initial_vertex_position, undeformed_tet_mesh) {
            // initialization
            // pre-compute constant values Dm_inv, dFdx and tet_volume for each tet element
            Dm_inv.clear();
            dFdx.clear();
            tet_volume.clear();
            for (int i = 0;i < this->undeformed_mesh_.NumOfElement();i++) {
                const Eigen::Matrix<int, 4, 1> elements = this->undeformed_mesh_.element(i);
                Eigen::Matrix<T, 3, 3> Dm;
                for (int j = 0;j < 3;j++)
                    Dm.col(j) = this->undeformed_mesh_.vertex(elements[j]) - this->undeformed_mesh_.vertex(elements[3]);
                Dm_inv.push_back(Dm.inverse());
                Eigen::Matrix<T, 9, 12> dFdx_i;
                dFdx_i.setZero();
                for (int j = 0;j < 4;j++)
                    for (int k = 0;k < 3;k++) {
                        Eigen::Matrix<T, 3, 3> dDsdx;
                        dDsdx.setZero();
                        if (j < 3) {
                            dDsdx(k, j) = 1;
                        } else {
                            for (int l = 0;l < 3;l++)
                                dDsdx(k, l) = -1;
                        }
                        Eigen::Matrix<T, 3, 3> dFdx_mat = dDsdx * Dm_inv[i];
                        for (int s = 0;s < 3;s++)
                            for (int t = 0;t < 3;t++)
                                dFdx_i(s * 3 + t, j * 3 + k) = dFdx_mat(t, s);
                    }
                dFdx.push_back(dFdx_i);
                tet_volume.push_back(std::fabs(Dm.determinant()) / 6.0);
            }
        }

        TetDeformableBody(const std::vector<std::reference_wrapper<const Material<3, T>>>& materials,
                    const std::vector<int>& material_id,
                    const Matrix3X<T>& initial_vertex_position,
                    const T density, const TetDeformableBody<T>& undeformed_tet_mesh)
                    : DeformableBody<3, T>(materials, material_id, initial_vertex_position, undeformed_tet_mesh) {}


        ~TetDeformableBody() {}

        const Matrix3X<T> ComputeElasticForce(const Matrix3X<T>& vertices) const {
            Matrix3X<T> force = MatrixX<T>::Zero(3, vertices.cols());
            const int tet_num = this->undeformed_mesh_.NumOfElement();
            for (int i = 0;i < tet_num;i++) {
                const Eigen::Matrix<int, 4, 1> elements = this->undeformed_mesh_.element(i);
                const Material<3, T>& material = this->materials_[this->material_id_[i]].get();
                Eigen::Matrix<T, 3, 3> Ds;  // deformed shape matrix
                for (int j = 0;j < 3;j++)
                    Ds.col(j) = vertices.col(elements[j]) - vertices.col(elements[3]);
                Eigen::Matrix<T, 3, 3> F = Ds * Dm_inv[i];
                Eigen::Matrix<T, 3, 3> P = material.StressTensor(F);
                Eigen::Matrix<T, 12, 1> df;
                df.setZero();
                for (int j = 0;j < 12;j++) {
                    for (int k = 0;k < 3;k++)
                        for (int l = 0;l < 3;l++)
                            df(j, 0) += P(k, l) * dFdx[i](l * 3 + k, j);
                }
                for (int j = 0;j < 4;j++)
                    force.col(elements[j]) -= df.segment(3 * j, 3) * tet_volume[i];
            }
            return force;
        }

        // TODO: HW4
        // Assignment 4, part 1.2
        // Compute stiffness matrix K for current vertices configuration
        // Hints:
        //      1. K is a 3N x 3N matrix, where N is the number of vertices.
        //      2. We've already computed the constant variables Ds and Dm_inv in class constructor code for you, feel free to use them.
        //      3. compute Ki in each tet element separetly (which is a 12x12 matrix), and sum up their contribution to the large K matrix.
        //      4. Be careful with the index mapping during assembling K.
        //      5. Read derivation carefully. Some matrix multifilication is mutiplying by element, be careful about that.
        const Eigen::SparseMatrix<T> ComputeStiffnessMatrix(const Matrix3X<T>& vertices) const {
            // triplet_list is a vector of triplet <row, col, value>
            std::vector<Eigen::Triplet<T> > triplet_list;
            triplet_list.clear();
            const int vertex_num = static_cast<int>(vertices.cols());
            const int tet_num = this->undeformed_mesh_.NumOfElement();
            // compute Ki for each tet element and fill the triplet_list vector
            for (int i = 0;i < tet_num;i++) {
                const Eigen::VectorXi elements = this->undeformed_mesh_.element(i);             // get the element of the tet
                const Material<3, T>& material = this->materials_[this->material_id_[i]].get(); // get the material model

                /* Implement your code here */
                
            }
            Eigen::SparseMatrix<T> K(vertex_num * 3, vertex_num * 3);
            // contruct sparse matrix K from triplet list, K(i, j) is sum of all value of triplet with row = i and col = j
            K.setFromTriplets(triplet_list.begin(), triplet_list.end());
            K = (K + Eigen::SparseMatrix<T>(K.transpose())) / 2.0;
            return K;
        }

        virtual void getInitialNodes(Matrix3X<T>& initial_nodes){
            initial_nodes = this->undeformed_mesh_.vertex();
        }

        std::vector<Eigen::Matrix<T, 3, 3> > Dm_inv;
        std::vector<Eigen::Matrix<T, 9, 12> > dFdx;
        std::vector<T> tet_volume;

    private:
        TetDeformableBody& operator=(const TetDeformableBody&);
    };
}