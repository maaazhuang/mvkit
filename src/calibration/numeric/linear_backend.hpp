#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include <span>

namespace mv::calibration::numeric {

enum class LinearBackendKind {
  Fallback,
  Eigen,
};

class LinearAlgebraBackend {
 public:
  virtual ~LinearAlgebraBackend() = default;

  virtual bool solve_symmetric(const std::vector<double>& matrix,
                               const std::vector<double>& rhs,
                               std::size_t n,
                               std::vector<double>* solution) const = 0;

  // 内部存储归调用方所有；回退后端保留既有求解路径。
  virtual bool solve_symmetric_into(std::span<const double> matrix,
                                    std::span<const double> rhs,
                                    std::span<double> solution) const;

  virtual bool smallest_right_singular_vector(const std::vector<double>& matrix,
                                               std::size_t rows,
                                               std::size_t cols,
                                               std::vector<double>* vector) const = 0;

  // 矩阵按行存储，特征向量位于列中；特征值和向量按特征值升序排列。
  virtual bool symmetric_eigen(const std::vector<double>& matrix,
                               std::size_t n,
                               std::vector<double>* eigenvalues,
                               std::vector<double>* eigenvectors) const = 0;
};

[[nodiscard]] std::unique_ptr<LinearAlgebraBackend> make_backend(LinearBackendKind kind);
[[nodiscard]] const LinearAlgebraBackend& shared_backend(LinearBackendKind kind);
[[nodiscard]] LinearBackendKind default_backend_kind() noexcept;
[[nodiscard]] bool eigen_backend_available() noexcept;

}  // namespace mv::calibration::numeric
