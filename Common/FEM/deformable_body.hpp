// Tao Du
// taodu@csail.mit.edu
// Sept 23, 2016
#pragma once
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "poly_mesh.hpp"
#include "typedefs.hpp"


namespace materials {
    template<int dim, typename T> class Material;

    template<int dim, typename T>
    class DeformableBody {
    public:
        // Type name convention: replace number in "Dim" in Vector2d or Vector3d.
        typedef Eigen::Matrix<T, dim, 1> VectorDimT;
        typedef Eigen::Matrix<T, dim, Eigen::Dynamic> MatrixDimXT;
        typedef PolyMesh<dim, T> PolyMeshDim;

        virtual ~DeformableBody() {}


        const MatrixDimXT& vertex_position() const { return vertex_position_; }


        virtual void getInitialNodes(MatrixDimXT& initial_nodes) = 0; //TODO: this is a terrible way to fix this, by having all the child classes implement this method




        // The return matrix must be symmetric and compressed. Assume A is the
        // stiffness matrix, then we can return (A.transpose() + A).makeCompressed(),
        // or use prune() to convert it into compressed format.
        virtual const Eigen::SparseMatrix<T> ComputeStiffnessMatrix(
                const MatrixDimXT& vertices) const = 0;

        /*
        virtual const T ComputeElasticEnergy(const MatrixDimXT& vertices) const = 0;
         */

        const PolyMeshDim& GetUndeformedMesh() const { return undeformed_mesh_; }

        void WriteMeshToFile(const std::string& file_name) const {
            (PolyMeshDim(vertex_position_, undeformed_mesh_.element(),
                         undeformed_mesh_.edge_in_element())).WriteToFile(file_name);
        }


    protected:
        // We allow users to specify different materials for each element. If a
        // single material is provided then materials_.size() = 1 and material_id_[i]
        // = 0 for all i. Otherwise materials[material_id_[i]] is the material in
        // element i.
        const std::vector<std::reference_wrapper<const Material<dim, T>>> materials_;
        const std::vector<int> material_id_;

        // Undeformed mesh.
        const PolyMeshDim& undeformed_mesh_;

        MatrixDimXT vertex_position_;





        DeformableBody(const Material<dim, T>& material,
                       const MatrixDimXT& vertex_position,
                       const PolyMeshDim& undeformed_mesh): materials_(1, material),
                                                            material_id_(undeformed_mesh.NumOfElement(), 0),
                                                            undeformed_mesh_(undeformed_mesh),
                                                            vertex_position_(vertex_position) {
        }


        DeformableBody(
                const std::vector<std::reference_wrapper<const Material<dim, T>>>& materials,
                const std::vector<int>& material_id,
                const MatrixDimXT& vertex_position,
                const PolyMeshDim& undeformed_mesh) : materials_(materials),
                                                      material_id_(material_id),
                                                      undeformed_mesh_(undeformed_mesh),
                                                      vertex_position_(vertex_position) {}



        DeformableBody(const DeformableBody<dim, T>& other): materials_(other.materials_),
                                                                       material_id_(other.material_id_),
                                                                       undeformed_mesh_(other.undeformed_mesh_),
                                                                       vertex_position_(other.vertex_position_) {}

    private:
        // Disable the copy assignment because we have constant data members.
        DeformableBody<dim, T>& operator=(const DeformableBody<dim, T>&);
    };

}

