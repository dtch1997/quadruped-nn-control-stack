#ifndef ACTIVATION_LEAKYRELU_H_
#define ACTIVATION_LEAKYRELU_H_

#include <Eigen/Core>
#include "../Config.h"

namespace MiniDNN
{


///
/// \ingroup Activations
///
/// The ReLU activation function
///
class LeakyReLU
{
    private:
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

    public:
        // a = activation(z) = max(z, 0)
        // Z = [z1, ..., zn], A = [a1, ..., an], n observations
        static inline void activate(const Matrix& Z, Matrix& A)
        {
            // A.array() = Z.array().cwiseMax(Scalar(0));
            A.array() = (Z.array() > Scalar(0)).select(Z, Z.array()*(Scalar(0.01)));
        }

        // Apply the Jacobian matrix J to a vector f
        // J = d_a / d_z = diag(sign(a)) = diag(a > 0)
        // g = J * f = (a > 0) .* f
        // Z = [z1, ..., zn], G = [g1, ..., gn], F = [f1, ..., fn]
        // Note: When entering this function, Z and G may point to the same matrix
        static inline void apply_jacobian(const Matrix& Z, const Matrix& A,
                                          const Matrix& F, Matrix& G)
        {
            G.array() = (A.array() > Scalar(0)).select(F, Scalar(0));
        }

        static std::string return_type()
        {
            return "LeakyReLU";
        }
};


} // namespace MiniDNN


#endif /* ACTIVATION_RELU_H_ */
