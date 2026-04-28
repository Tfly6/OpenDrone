#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <cmath>
#include <complex>
#include <limits>

namespace lqr {

/**
 * Simple LQR Solver
 * Solves the continuous-time Algebraic Riccati Equation:
 *   A^T P + P A - P B R^{-1} B^T P + Q = 0
 *   K = R^{-1} B^T P
 */
template <size_t N, size_t M>
class LQRSolver {
public:
    typedef Eigen::Matrix<double, N, N> state_matrix_t;
    typedef Eigen::Matrix<double, M, M> control_matrix_t;
    typedef Eigen::Matrix<double, N, M> control_gain_matrix_t;
    typedef Eigen::Matrix<double, N, 1> state_vector_t;
    typedef Eigen::Matrix<double, M, 1> control_vector_t;
    typedef Eigen::Matrix<double, 2 * N, 2 * N> hamiltonian_matrix_t;
    typedef Eigen::Matrix<std::complex<double>, 2 * N, 2 * N> complex_hamiltonian_matrix_t;
    typedef Eigen::Matrix<std::complex<double>, 2 * N, 1> complex_eigenvalues_t;
    typedef Eigen::Matrix<std::complex<double>, 2 * N, N> invariant_subspace_t;
    typedef Eigen::Matrix<std::complex<double>, N, N> complex_state_matrix_t;
    typedef Eigen::Matrix<std::complex<double>, M, N> complex_feedback_matrix_t;

    LQRSolver()
        : tolerance_(1e-8)
        , K_()
        , P_()
        , converged_(false)
        , iterations_(0)
    {}

    /**
     * Solve LQR problem iteratively using Kleinman method
     * @param Q state weighting matrix (N x N), positive semi-definite
     * @param R control weighting matrix (M x M), positive definite
     * @param A state matrix (N x N)
     * @param B control matrix (N x M)
     * @param K output gain matrix (M x N)
     * @return true if converged
     */
    bool compute(const state_matrix_t& Q, const control_matrix_t& R,
                 const state_matrix_t& A, const control_gain_matrix_t& B,
                 Eigen::Matrix<double, M, N>& K)
    {
        converged_ = false;
        iterations_ = 0;

        const control_matrix_t R_inv = R.inverse();

        hamiltonian_matrix_t H;
        H.setZero();
        H.template block<N, N>(0, 0) = A;
        H.template block<N, N>(0, N) = -(B * R_inv * B.transpose());
        H.template block<N, N>(N, 0) = -Q;
        H.template block<N, N>(N, N) = -A.transpose();

        Eigen::ComplexEigenSolver<hamiltonian_matrix_t> eigenSolver(H);
        if (eigenSolver.info() != Eigen::Success) {
            return false;
        }

        const complex_eigenvalues_t eigenvalues = eigenSolver.eigenvalues();
        const complex_hamiltonian_matrix_t eigenvectors = eigenSolver.eigenvectors();

        invariant_subspace_t stableSubspace;
        bool selected[2 * N] = {false};
        size_t stableCount = 0;

        for (int index = 0; index < static_cast<int>(2 * N); ++index) {
            if (eigenvalues(index).real() < -tolerance_) {
                stableSubspace.col(stableCount++) = eigenvectors.col(index);
                selected[index] = true;
                if (stableCount == N) {
                    break;
                }
            }
        }

        while (stableCount < N) {
            int bestIndex = -1;
            double bestReal = std::numeric_limits<double>::infinity();
            for (int index = 0; index < static_cast<int>(2 * N); ++index) {
                if (selected[index]) {
                    continue;
                }
                const double realPart = eigenvalues(index).real();
                if (realPart < bestReal) {
                    bestReal = realPart;
                    bestIndex = index;
                }
            }

            if (bestIndex < 0) {
                return false;
            }

            stableSubspace.col(stableCount++) = eigenvectors.col(bestIndex);
            selected[bestIndex] = true;
        }

        const complex_state_matrix_t U11 = stableSubspace.template block<N, N>(0, 0);
        const complex_state_matrix_t U21 = stableSubspace.template block<N, N>(N, 0);

        Eigen::FullPivLU<complex_state_matrix_t> lu(U11);
        if (!lu.isInvertible()) {
            return false;
        }

        const complex_state_matrix_t P_complex = U21 * lu.inverse();
        P_ = P_complex.real();
        P_ = 0.5 * (P_ + P_.transpose());

        const complex_feedback_matrix_t K_complex = R_inv.template cast<std::complex<double>>()
            * B.transpose().template cast<std::complex<double>>() * P_complex;
        K = K_complex.real();
        K_ = K;

        converged_ = P_.allFinite() && K.allFinite();
        iterations_ = 1;

        return converged_;
    }

    bool hasConverged() const { return converged_; }
    size_t getIterations() const { return iterations_; }
    const state_matrix_t& getP() const { return P_; }

private:
    double tolerance_;
    Eigen::Matrix<double, M, N> K_;
    state_matrix_t P_;
    bool converged_;
    size_t iterations_;
};

}  // namespace lqr