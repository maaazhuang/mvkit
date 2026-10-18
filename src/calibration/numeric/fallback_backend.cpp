#include <calibration/numeric/linear_backend.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

namespace mv::calibration::numeric {
namespace {

bool gaussian_solve_work(std::span<double> a, std::span<double> b,
                    std::size_t n, std::vector<double>* x) {
  if (a.size() != n * n || b.size() != n || x == nullptr || n == 0) return false;
  x->assign(n, 0.0);
  for (std::size_t k = 0; k < n; ++k) {
    std::size_t pivot = k;
    double best = std::abs(a[k * n + k]);
    for (std::size_t r = k + 1; r < n; ++r) {
      const double candidate = std::abs(a[r * n + k]);
      if (candidate > best) {
        best = candidate;
        pivot = r;
      }
    }
    if (!std::isfinite(best) || best < 1e-18) return false;
    if (pivot != k) {
      for (std::size_t c = k; c < n; ++c) std::swap(a[k * n + c], a[pivot * n + c]);
      std::swap(b[k], b[pivot]);
    }
    const double diagonal = a[k * n + k];
    for (std::size_t r = k + 1; r < n; ++r) {
      const double factor = a[r * n + k] / diagonal;
      a[r * n + k] = 0.0;
      for (std::size_t c = k + 1; c < n; ++c) a[r * n + c] -= factor * a[k * n + c];
      b[r] -= factor * b[k];
    }
  }
  for (std::size_t row = n; row-- > 0;) {
    double value = b[row];
    for (std::size_t c = row + 1; c < n; ++c) value -= a[row * n + c] * (*x)[c];
    const double diagonal = a[row * n + row];
    if (!std::isfinite(diagonal) || std::abs(diagonal) < 1e-18) return false;
    (*x)[row] = value / diagonal;
    if (!std::isfinite((*x)[row])) return false;
  }
  return true;
}

template <std::size_t FixedSize = 0>
bool jacobi_eigen_work(std::span<double> matrix, std::size_t dimension,
                  std::vector<double>* values, std::vector<double>* vectors) {
  const std::size_t n = FixedSize ? FixedSize : dimension;
  if (n == 0 || matrix.size() != n * n || values == nullptr || vectors == nullptr) return false;
  vectors->assign(n * n, 0.0);
  for (std::size_t i = 0; i < n; ++i) (*vectors)[i * n + i] = 1.0;

  const std::size_t max_iterations = std::max<std::size_t>(64, 80 * n * n);
  for (std::size_t iteration = 0; iteration < max_iterations; ++iteration) {
    std::size_t p = 0;
    std::size_t q = 0;
    double largest = 0.0;
    for (std::size_t r = 0; r < n; ++r) {
      for (std::size_t c = r + 1; c < n; ++c) {
        const double magnitude = std::abs(matrix[r * n + c]);
        if (magnitude > largest) {
          largest = magnitude;
          p = r;
          q = c;
        }
      }
    }
    if (largest <= 1e-14) break;
    const double app = matrix[p * n + p];
    const double aqq = matrix[q * n + q];
    const double apq = matrix[p * n + q];
    const double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
    const double c = std::cos(phi);
    const double s = std::sin(phi);
    for (std::size_t k = 0; k < n; ++k) {
      if (k == p || k == q) continue;
      const double mkp = matrix[k * n + p];
      const double mkq = matrix[k * n + q];
      const double new_kp = c * mkp - s * mkq;
      const double new_kq = s * mkp + c * mkq;
      matrix[k * n + p] = new_kp;
      matrix[p * n + k] = new_kp;
      matrix[k * n + q] = new_kq;
      matrix[q * n + k] = new_kq;
    }
    matrix[p * n + p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
    matrix[q * n + q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
    matrix[p * n + q] = 0.0;
    matrix[q * n + p] = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
      const double vkp = (*vectors)[k * n + p];
      const double vkq = (*vectors)[k * n + q];
      (*vectors)[k * n + p] = c * vkp - s * vkq;
      (*vectors)[k * n + q] = s * vkp + c * vkq;
    }
  }

  values->resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    (*values)[i] = matrix[i * n + i];
    if (!std::isfinite((*values)[i])) return false;
  }

  if (n <= 16) {
    // 常用的小矩阵复用栈上工作区，保持 Jacobi 次序与相等特征值的稳定排序。
    std::array<std::size_t, 16> order;
    std::array<double, 16> old_values;
    std::array<double, 256> old_vectors;
    std::iota(order.begin(), order.begin() + n, 0);
    std::copy_n(values->begin(), n, old_values.begin());
    std::copy_n(vectors->begin(), n * n, old_vectors.begin());
    for (std::size_t i = 1; i < n; ++i) {
      const auto index = order[i];
      auto position = i;
      while (position && old_values[index] < old_values[order[position - 1]]) {
        order[position] = order[position - 1];
        --position;
      }
      order[position] = index;
    }
    for (std::size_t column = 0; column < n; ++column) {
      (*values)[column] = old_values[order[column]];
      for (std::size_t row = 0; row < n; ++row)
        (*vectors)[row * n + column] = old_vectors[row * n + order[column]];
    }
    return true;
  }
  std::vector<std::size_t> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t lhs, std::size_t rhs) {
    return (*values)[lhs] < (*values)[rhs];
  });
  const auto old_values = *values;
  const auto old_vectors = *vectors;
  for (std::size_t new_index = 0; new_index < n; ++new_index) {
    const auto old_index = order[new_index];
    (*values)[new_index] = old_values[old_index];
    for (std::size_t row = 0; row < n; ++row) {
      (*vectors)[row * n + new_index] = old_vectors[row * n + old_index];
    }
  }
  return true;
}

bool jacobi_eigen(const std::vector<double>& matrix, std::size_t n,
                  std::vector<double>* values, std::vector<double>* vectors) {
  if (!n || matrix.size() != n * n || !values || !vectors) return false;
  if (n <= 16) {
    std::array<double, 256> working;
    std::copy(matrix.begin(), matrix.end(), working.begin());
    const std::span<double> view{working.data(), n * n};
    // 常用维数在编译期固定；算术和扫描顺序不变，消除动态步长及循环控制。
    if (n == 3) return jacobi_eigen_work<3>(view, n, values, vectors);
    if (n == 6) return jacobi_eigen_work<6>(view, n, values, vectors);
    if (n == 10) return jacobi_eigen_work<10>(view, n, values, vectors);
    return jacobi_eigen_work(view, n, values, vectors);
  }
  auto working = matrix;
  return jacobi_eigen_work(std::span<double>{working}, n, values, vectors);
}

bool gaussian_solve(const std::vector<double>& matrix, const std::vector<double>& rhs,
                    std::size_t n, std::vector<double>* solution) {
  if (!n || matrix.size() != n * n || rhs.size() != n || !solution) return false;
  if (n <= 16) {
    std::array<double, 256> a;
    std::array<double, 16> b;
    std::copy(matrix.begin(), matrix.end(), a.begin());
    std::copy(rhs.begin(), rhs.end(), b.begin());
    return gaussian_solve_work({a.data(), n * n}, {b.data(), n}, n, solution);
  }
  auto a = matrix;
  auto b = rhs;
  return gaussian_solve_work(a, b, n, solution);
}

class FallbackBackend final : public LinearAlgebraBackend {
 public:
  bool solve_symmetric(const std::vector<double>& matrix, const std::vector<double>& rhs,
                       std::size_t n, std::vector<double>* solution) const override {
    return gaussian_solve(matrix, rhs, n, solution);
  }

  bool smallest_right_singular_vector(const std::vector<double>& matrix,
                                       std::size_t rows, std::size_t cols,
                                       std::vector<double>* vector) const override {
    if (rows == 0 || cols == 0 || matrix.size() != rows * cols || vector == nullptr) return false;
    std::vector<double> gram(cols * cols, 0.0);
    for (std::size_t r = 0; r < rows; ++r) {
      for (std::size_t c = 0; c < cols; ++c) {
        for (std::size_t k = 0; k < cols; ++k) {
          gram[c * cols + k] += matrix[r * cols + c] * matrix[r * cols + k];
        }
      }
    }
    std::vector<double> values;
    std::vector<double> vectors;
    if (!jacobi_eigen(gram, cols, &values, &vectors)) return false;
    vector->resize(cols);
    double norm_squared = 0.0;
    for (std::size_t row = 0; row < cols; ++row) {
      (*vector)[row] = vectors[row * cols];
      norm_squared += (*vector)[row] * (*vector)[row];
    }
    if (!std::isfinite(norm_squared) || norm_squared <= std::numeric_limits<double>::epsilon()) return false;
    const double inverse_norm = 1.0 / std::sqrt(norm_squared);
    for (double& value : *vector) value *= inverse_norm;
    return true;
  }

  bool symmetric_eigen(const std::vector<double>& matrix, std::size_t n,
                       std::vector<double>* eigenvalues,
                       std::vector<double>* eigenvectors) const override {
    return jacobi_eigen(matrix, n, eigenvalues, eigenvectors);
  }
};

}  // namespace

std::unique_ptr<LinearAlgebraBackend> make_fallback_backend() {
  return std::make_unique<FallbackBackend>();
}

}  // namespace mv::calibration::numeric
