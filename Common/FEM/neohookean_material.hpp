// Tao Du
// taodu@csail.mit.edu
// Sept 23, 2016
#pragma once
#include "material.hpp"
#include <iostream>

namespace materials {

template<int dim, typename T>
class NeohookeanElasticityMaterial : public Material<dim, T> {
public:
    NeohookeanElasticityMaterial(const T young_modulus,
        const T poisson_ratio)
        : Material<dim, T>(young_modulus, poisson_ratio) {}

    NeohookeanElasticityMaterial(
        const NeohookeanElasticityMaterial<dim, T>& material)
        : Material<dim, T>(material) {}

    ~NeohookeanElasticityMaterial() {}

    const T EnergyDensity(const typename Material<dim, T>::MatrixDimT& F) const {
        const T I1 = (F.transpose() * F).trace();
        const T J = F.determinant();
        const T log_J = log(J);
        const T mu = Material<dim, T>::mu();
        const T lambda = Material<dim, T>::lambda();
        return mu / 2.0 * (I1 - dim) - mu * log_J +
                lambda / 2.0 * log_J * log_J;
    }

    const typename Material<dim, T>::MatrixDimT StressTensor(const typename Material<dim, T>::MatrixDimT& F) const {
        const T J = F.determinant();
        const typename Material<dim, T>::MatrixDimT F_inv_trans = F.inverse().transpose();
        const T mu = Material<dim, T>::mu();
        const T lambda = Material<dim, T>::lambda();
        return mu * (F - F_inv_trans) + lambda * log(J) * F_inv_trans;
    }

    const typename Material<dim, T>::MatrixDimT StressDifferential(
        const typename Material<dim, T>::MatrixDimT& F,
        const typename Material<dim, T>::MatrixDimT& dF) const {
        
        const T J = F.determinant();
        const typename Material<dim, T>::MatrixDimT F_inv = F.inverse();
        const typename Material<dim, T>::MatrixDimT F_inv_trans = F_inv.transpose();
        const T mu = Material<dim, T>::mu();
        const T lambda = Material<dim, T>::lambda();
        return mu * dF
            + (mu - lambda * log(J)) * F_inv_trans * dF.transpose()
            * F_inv_trans
            + lambda * (F_inv * dF).trace() * F_inv_trans;
    }

    const typename Material<dim, T>::MatrixDim2T StressDifferential(
        const typename Material<dim, T>::MatrixDimT& F) const {
        
        const T mu = Material<dim, T>::mu();
        const T lambda = Material<dim, T>::lambda();
        typename Material<dim, T>::MatrixDim2T K =
            Material<dim, T>::MatrixDim2T::Identity() * mu;
        const T J = F.determinant();
        const typename Material<dim, T>::MatrixDimT F_inv = F.inverse();
        const typename Material<dim, T>::MatrixDimT F_inv_trans = F_inv.transpose();
        const Eigen::Matrix<T, dim * dim, 1> f_inv_trans =
            Eigen::Map<const Eigen::Matrix<T, dim * dim, 1>>(
            F_inv_trans.data(), dim * dim);
        K += lambda * f_inv_trans * f_inv_trans.transpose();
        const T scale = (mu - lambda * log(J));
        for (int i = 0; i < dim; ++i)
            for (int j = 0; j < dim; ++j)
            for (int k = 0; k < dim; ++k)
                for (int s = 0; s < dim; ++s)
                K(i + j * dim, k + s * dim) += scale * F_inv_trans(i, s)
                    * F_inv_trans(k, j);
        return K;
    }

private:
  NeohookeanElasticityMaterial<dim, T>& operator=(
    const NeohookeanElasticityMaterial<dim, T>&);
};

}