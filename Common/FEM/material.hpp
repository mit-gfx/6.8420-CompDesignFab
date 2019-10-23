#pragma once

#include <Eigen/Core>

namespace materials {

    template<int dim, typename T>
    class Material {
    protected:
        // Type name convention: replace number with "Dim" in Matrix2d or Matrix3d.
        typedef Eigen::Matrix<T, dim, dim> MatrixDimT;
        typedef Eigen::Matrix<T, dim * dim, dim * dim> MatrixDim2T;

    public:
        Material(const T young_modulus, const T poisson_ratio)
                : young_modulus_(young_modulus), poisson_ratio_(poisson_ratio),
                  mu_(young_modulus / 2.0 / (1 + poisson_ratio)),
                  lambda_(young_modulus * poisson_ratio /
                          (1 + poisson_ratio) / (1 - 2 * poisson_ratio)) {}


        Material(const Material<dim, T>& material)
                : young_modulus_(material.young_modulus_),
                  poisson_ratio_(material.poisson_ratio_),
                  mu_(material.young_modulus_ / 2.0 / (1 + material.poisson_ratio_)),
                  lambda_(material.young_modulus_ * material.poisson_ratio_ /
                          (1 + material.poisson_ratio_) / (1 - 2 * material.poisson_ratio_)) {}
        virtual ~Material() {}
        const double young_modulus() const { return young_modulus_; }
        const double poisson_ratio() const { return poisson_ratio_; }
        const double mu() const { return mu_; }
        const double lambda() const { return lambda_; }


        virtual const T EnergyDensity(const MatrixDimT& F) const = 0;
        virtual const MatrixDimT StressTensor(const MatrixDimT& F) const = 0;
        virtual const MatrixDimT StressDifferential(const MatrixDimT& F,
                                                    const MatrixDimT& dF) const = 0;

        virtual const MatrixDim2T StressDifferential(const MatrixDimT& F) const = 0;

    private:
        // Intentionally disable the copy assignment since all data members are
        // constant. Do not provide the definition so that we won't be able to
        // accidentally use it in member functions.
        Material<dim, T>& operator=(const Material<dim, T>&);

        const double young_modulus_;
        const double poisson_ratio_;
        const double mu_;
        const double lambda_;
    };

}

