#include <calibration/calibration/quality.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <calibration/numeric/linear_backend.hpp>
#include <core/exception.hpp>

namespace mv::calibration {
namespace {

struct LayoutEntry {
  const solver::ParameterBlock *block{};
  std::size_t offset{};
};

struct Evaluation {
  std::size_t rows{};
  std::size_t cols{};
  std::vector<double> residuals;
  std::vector<double> jacobian;
};

Evaluation evaluate_with_jacobian(const solver::SolverProblem &problem) {
  std::unordered_map<solver::ParameterBlockId, const solver::ParameterBlock *>
      by_id;
  std::unordered_map<solver::ParameterBlockId, std::size_t> active_offset;
  std::size_t cols = 0;
  for (const auto &block : problem.parameter_blocks) {
    if (!by_id.emplace(block.id, &block).second) {
      throw core::Exception(core::Status::InvalidState,
                            "compute_calibration_uncertainty",
                            "duplicate parameter block id");
    }
    if (!block.constant) {
      active_offset.emplace(block.id, cols);
      cols += block.tangent_size;
    }
  }

  std::size_t rows = 0;
  for (const auto &residual : problem.residual_blocks) {
    if (!residual.function) {
      throw core::Exception(core::Status::InvalidState,
                            "compute_calibration_uncertainty",
                            "null residual function");
    }
    rows += residual.function->residual_size();
  }

  Evaluation result;
  result.rows = rows;
  result.cols = cols;
  result.residuals.assign(rows, 0.0);
  result.jacobian.assign(rows * cols, 0.0);

  std::size_t row_base = 0;
  for (const auto &residual_block : problem.residual_blocks) {
    const auto ids = residual_block.function->parameter_blocks();
    const auto residual_size = residual_block.function->residual_size();
    std::vector<solver::ParameterBlockView> parameter_views;
    std::vector<std::vector<double>> local_storage;
    std::vector<solver::JacobianView> local_views;
    parameter_views.reserve(ids.size());
    local_storage.reserve(ids.size());
    local_views.reserve(ids.size());

    for (const auto id : ids) {
      const auto it = by_id.find(id);
      if (it == by_id.end()) {
        throw core::Exception(core::Status::InvalidState,
                              "compute_calibration_uncertainty",
                              "residual references unknown parameter block");
      }
      const auto &block = *it->second;
      parameter_views.push_back(
          {block.id, block.values, block.tangent_size, block.manifold});
      local_storage.emplace_back(residual_size * block.tangent_size, 0.0);
      auto &storage = local_storage.back();
      local_views.push_back({storage, residual_size, block.tangent_size});
    }

    std::span<double> residual_output(result.residuals.data() + row_base,
                                      residual_size);
    const auto status = residual_block.function->evaluate(
        parameter_views, residual_output, local_views);
    if (status != core::Status::Ok) {
      throw core::Exception(status, "compute_calibration_uncertainty",
                            "residual/Jacobian evaluation failed");
    }

    for (std::size_t parameter_index = 0; parameter_index < ids.size();
         ++parameter_index) {
      const auto active = active_offset.find(ids[parameter_index]);
      if (active == active_offset.end())
        continue;
      const auto &local = local_views[parameter_index];
      for (std::size_t r = 0; r < local.rows; ++r) {
        for (std::size_t c = 0; c < local.cols; ++c) {
          const double value = local(r, c);
          if (!std::isfinite(value)) {
            throw core::Exception(core::Status::InvalidState,
                                  "compute_calibration_uncertainty",
                                  "non-finite Jacobian");
          }
          result.jacobian[(row_base + r) * cols + active->second + c] = value;
        }
      }
    }
    row_base += residual_size;
  }

  return result;
}

struct CovarianceComputation {
  bool available{false};
  std::vector<double> covariance;
  std::string message;
};

// 用保守范数界证明完整系统的同一秩条件；块结构或界不确定时使用特征求解器。
bool arrowhead_covariance(const std::vector<double> &a, std::size_t n,
                          std::size_t global,
                          const numeric::LinearAlgebraBackend &backend,
                          std::vector<double> *inverse) {
  if (global == 0 || global > 16 || n <= global || (n - global) % 6 != 0)
    return false;
  const auto poses = (n - global) / 6;
  if (poses < 4)
    return false;
  for (std::size_t p = 0; p < poses; ++p)
    for (std::size_t q = 0; q < p; ++q)
      for (std::size_t i = 0; i < 6; ++i)
        for (std::size_t j = 0; j < 6; ++j)
          if (a[(global + 6 * p + i) * n + global + 6 * q + j] != 0)
            return false;
  std::vector<double> d(poses * 36), e(poses * 6 * global), s(global * global);
  for (std::size_t i = 0; i < global; ++i)
    for (std::size_t j = 0; j < global; ++j)
      s[i * global + j] = a[i * n + j];
  for (std::size_t p = 0; p < poses; ++p) {
    const auto start = global + 6 * p;
    double l[6][6]{};
    for (std::size_t i = 0; i < 6; ++i)
      for (std::size_t j = 0; j <= i; ++j) {
        double v = a[(start + i) * n + start + j];
        for (std::size_t k = 0; k < j; ++k)
          v -= l[i][k] * l[j][k];
        if (i == j) {
          if (!(v > 0) || !std::isfinite(v))
            return false;
          l[i][j] = std::sqrt(v);
        } else
          l[i][j] = v / l[j][j];
      }
    for (std::size_t c = 0; c < 6; ++c) {
      double x[6]{};
      for (std::size_t i = 0; i < 6; ++i) {
        double v = i == c ? 1. : 0.;
        for (std::size_t k = 0; k < i; ++k)
          v -= l[i][k] * x[k];
        x[i] = v / l[i][i];
      }
      for (std::size_t i = 6; i-- > 0;) {
        double v = x[i];
        for (std::size_t k = i + 1; k < 6; ++k)
          v -= l[k][i] * x[k];
        x[i] = v / l[i][i];
      }
      for (std::size_t i = 0; i < 6; ++i)
        d[p * 36 + i * 6 + c] = x[i];
    }
    for (std::size_t i = 0; i < 6; ++i)
      for (std::size_t c = 0; c < global; ++c)
        for (std::size_t k = 0; k < 6; ++k)
          e[(p * 6 + i) * global + c] +=
              d[p * 36 + i * 6 + k] * a[(start + k) * n + c];
    for (std::size_t i = 0; i < global; ++i)
      for (std::size_t j = 0; j < global; ++j)
        for (std::size_t k = 0; k < 6; ++k)
          s[i * global + j] -=
              a[i * n + start + k] * e[(p * 6 + k) * global + j];
  }
  // 正定性证明要求 Schur 补为正。
  std::vector<double> l(global * global);
  for (std::size_t i = 0; i < global; ++i)
    for (std::size_t j = 0; j <= i; ++j) {
      double v = s[i * global + j];
      for (std::size_t k = 0; k < j; ++k)
        v -= l[i * global + k] * l[j * global + k];
      if (i == j) {
        if (!(v > 0) || !std::isfinite(v))
          return false;
        l[i * global + j] = std::sqrt(v);
      } else
        l[i * global + j] = v / l[j * global + j];
    }
  std::vector<double> si(global * global), rhs(global), x;
  for (std::size_t c = 0; c < global; ++c) {
    std::fill(rhs.begin(), rhs.end(), 0);
    rhs[c] = 1;
    if (!backend.solve_symmetric(s, rhs, global, &x))
      return false;
    for (std::size_t i = 0; i < global; ++i)
      si[i * global + c] = x[i];
  }
  inverse->assign(n * n, 0);
  for (std::size_t i = 0; i < global; ++i)
    for (std::size_t j = 0; j < global; ++j)
      (*inverse)[i * n + j] = si[i * global + j];
  std::vector<double> es((n - global) * global);
  for (std::size_t i = 0; i < n - global; ++i)
    for (std::size_t j = 0; j < global; ++j)
      for (std::size_t k = 0; k < global; ++k)
        es[i * global + j] += e[i * global + k] * si[k * global + j];
  for (std::size_t i = 0; i < n - global; ++i) {
    for (std::size_t j = 0; j < global; ++j)
      (*inverse)[(global + i) * n + j] = (*inverse)[j * n + global + i] =
          -es[i * global + j];
    for (std::size_t j = 0; j < n - global; ++j) {
      double v = i / 6 == j / 6 ? d[(i / 6) * 36 + (i % 6) * 6 + j % 6] : 0.;
      for (std::size_t k = 0; k < global; ++k)
        v += es[i * global + k] * e[j * global + k];
      (*inverse)[(global + i) * n + global + j] = v;
    }
  }
  double norm = 0, inverse_norm = 0;
  for (std::size_t i = 0; i < n; ++i) {
    double row = 0, invrow = 0;
    for (std::size_t j = 0; j < n; ++j) {
      row += std::abs(a[i * n + j]);
      invrow += std::abs((*inverse)[i * n + j]);
    }
    norm = std::max(norm, row);
    inverse_norm = std::max(inverse_norm, invrow);
  }
  // 以 ||I-A*X||_inf 认证计算逆；Neumann 界给出 ||A^-1|| <= ||X||/(1-residual)，不确定时回退。
  if (!std::isfinite(inverse_norm) || inverse_norm <= 0)
    return false;
  double residual_norm = 0;
  std::vector<std::size_t> nonzero;
  for (std::size_t i = 0; i < n; ++i) {
    nonzero.clear();
    for (std::size_t k = 0; k < n; ++k)
      if (a[i * n + k] != 0)
        nonzero.push_back(k);
    double row = 0;
    for (std::size_t j = 0; j < n; ++j) {
      double v = 0;
      for (auto k : nonzero)
        v += a[i * n + k] * (*inverse)[k * n + j];
      row += std::abs((i == j ? 1. : 0.) - v);
    }
    residual_norm = std::max(residual_norm, row);
  }
  return std::isfinite(residual_norm) && residual_norm < .01 &&
         (1 - residual_norm) / inverse_norm >
             16 * std::max(1e-12 * norm, 1e-14);
}

CovarianceComputation
covariance_from_evaluation(const Evaluation &evaluation,
                           const numeric::LinearAlgebraBackend &backend,
                           std::span<const std::size_t> requested) {
  CovarianceComputation output;
  const std::size_t m = evaluation.rows;
  const std::size_t n = evaluation.cols;
  if (m == 0 || n == 0 || m < n) {
    output.message = "covariance unavailable: underdetermined local system";
    return output;
  }

  std::vector<double> column_scale(n, 0.0);
  for (std::size_t c = 0; c < n; ++c) {
    double norm_squared = 0.0;
    for (std::size_t r = 0; r < m; ++r) {
      const double value = evaluation.jacobian[r * n + c];
      norm_squared += value * value;
    }
    column_scale[c] = std::sqrt(norm_squared);
    if (!std::isfinite(column_scale[c]) || column_scale[c] <= 1e-16) {
      output.message = "covariance unavailable: rank-deficient Jacobian";
      return output;
    }
  }

  std::vector<double> gram(n * n, 0.0);
  std::vector<std::size_t> nonzero;
  nonzero.reserve(n);
  for (std::size_t r = 0; r < m; ++r) {
    nonzero.clear();
    for (std::size_t c = 0; c < n; ++c) {
      const double jc = evaluation.jacobian[r * n + c] / column_scale[c];
      if (jc == 0)
        continue;
      nonzero.push_back(c);
      for (const auto k : nonzero) {
        gram[c * n + k] +=
            jc * (evaluation.jacobian[r * n + k] / column_scale[k]);
      }
    }
  }
  for (std::size_t c = 0; c < n; ++c) {
    for (std::size_t k = 0; k < c; ++k)
      gram[k * n + c] = gram[c * n + k];
  }

  std::vector<double> normalized_inverse;
  const auto globals =
      requested.empty()
          ? 0
          : *std::max_element(requested.begin(), requested.end()) + 1;
  if (!arrowhead_covariance(gram, n, globals, backend, &normalized_inverse)) {
    std::vector<double> eigenvalues;
    std::vector<double> eigenvectors;
    if (!backend.symmetric_eigen(gram, n, &eigenvalues, &eigenvectors)) {
      output.message = "covariance unavailable: eigen decomposition failed";
      return output;
    }

    double largest = 0.0;
    for (double value : eigenvalues)
      largest = std::max(largest, std::abs(value));
    const double threshold = std::max(1e-12 * largest, 1e-14);
    for (double value : eigenvalues) {
      if (value <= threshold) {
        output.message = "covariance unavailable: rank-deficient Jacobian";
        return output;
      }
    }

    normalized_inverse.assign(n * n, 0.0);
    for (std::size_t eigen_index = 0; eigen_index < n; ++eigen_index) {
      const double inverse_value = 1.0 / eigenvalues[eigen_index];
      for (const auto r : requested) {
        const double vr = eigenvectors[r * n + eigen_index];
        for (const auto c : requested) {
          normalized_inverse[r * n + c] +=
              inverse_value * vr * eigenvectors[c * n + eigen_index];
        }
      }
    }
  }

  double sse = 0.0;
  for (double residual : evaluation.residuals)
    sse += residual * residual;
  const auto dof = std::max<std::size_t>(1, m - n);
  const double sigma_squared = sse / static_cast<double>(dof);

  output.covariance.resize(n * n, 0.0);
  for (std::size_t r = 0; r < n; ++r) {
    for (std::size_t c = 0; c < n; ++c) {
      output.covariance[r * n + c] = sigma_squared *
                                     normalized_inverse[r * n + c] /
                                     (column_scale[r] * column_scale[c]);
    }
  }
  output.available = true;
  output.message = "covariance available";
  return output;
}

} // namespace

void compute_calibration_uncertainty(
    const solver::SolverProblem &problem,
    std::span<const IntrinsicBinding> camera_bindings,
    CalibrationResult *result, numeric::LinearBackendKind backend_kind) {
  if (result == nullptr) {
    throw core::Exception(core::Status::InvalidArgument,
                          "compute_calibration_uncertainty",
                          "result output is null");
  }

  result->camera_parameter_names.clear();
  result->camera_parameter_names.reserve(camera_bindings.size());
  for (const auto &binding : camera_bindings)
    result->camera_parameter_names.push_back(binding.name);
  const std::size_t camera_count = camera_bindings.size();
  result->camera_stddev.assign(camera_count, 0.0);
  result->camera_covariance.assign(camera_count * camera_count, 0.0);
  result->covariance_available = false;

  std::map<solver::ParameterBlockId, std::size_t> active_offset;
  std::map<solver::ParameterBlockId, const solver::ParameterBlock *> by_id;
  std::size_t offset = 0;
  for (const auto &block : problem.parameter_blocks) {
    by_id.emplace(block.id, &block);
    if (!block.constant) {
      active_offset.emplace(block.id, offset);
      offset += block.tangent_size;
    }
  }

  std::vector<std::size_t> requested;
  for (const auto &binding : camera_bindings)
    if (!by_id.at(binding.id)->constant)
      requested.push_back(active_offset.at(binding.id));
  const auto evaluation = evaluate_with_jacobian(problem);
  auto backend = numeric::make_backend(backend_kind);
  const auto covariance =
      covariance_from_evaluation(evaluation, *backend, requested);
  result->quality_message = covariance.message;
  if (!covariance.available)
    return;

  for (std::size_t r = 0; r < camera_count; ++r) {
    const auto *row_block = by_id.at(camera_bindings[r].id);
    if (row_block->constant)
      continue;
    const auto row_offset = active_offset.at(row_block->id);
    const double variance =
        covariance.covariance[row_offset * evaluation.cols + row_offset];
    result->camera_stddev[r] =
        camera_bindings[r].scale * std::sqrt(std::max(0.0, variance));
    for (std::size_t c = 0; c < camera_count; ++c) {
      const auto *col_block = by_id.at(camera_bindings[c].id);
      if (col_block->constant)
        continue;
      const auto col_offset = active_offset.at(col_block->id);
      result->camera_covariance[r * camera_count + c] =
          camera_bindings[r].scale * camera_bindings[c].scale *
          covariance.covariance[row_offset * evaluation.cols + col_offset];
    }
  }
  result->covariance_available = true;
}

} // namespace mv::calibration
