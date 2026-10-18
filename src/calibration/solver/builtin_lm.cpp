#include <calibration/solver/builtin_lm.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include <calibration/numeric/linear_backend.hpp>
#include <calibration/solver/manifold.hpp>

namespace mv::calibration::solver {
namespace {

struct LayoutEntry {
  ParameterBlock *block{};
  std::size_t offset{};
};

struct Evaluation {
  struct RowBlock {
    std::size_t begin{}, end{};
    std::vector<std::size_t> columns;
  };
  std::vector<RowBlock> blocks;
  double cost{};
  std::vector<double> residuals;
  std::vector<double> jacobian;
  std::size_t rows{};
  std::size_t cols{};
};

core::Status
build_layout(SolverProblem &problem, std::vector<LayoutEntry> *layout,
             std::unordered_map<ParameterBlockId, std::size_t> *index_by_id,
             std::size_t *active_size) {
  layout->clear();
  index_by_id->clear();
  *active_size = 0;
  for (std::size_t i = 0; i < problem.parameter_blocks.size(); ++i) {
    auto &block = problem.parameter_blocks[i];
    if (validate_parameter_block(block) != core::Status::Ok)
      return core::Status::InvalidArgument;
    if (!index_by_id->emplace(block.id, i).second)
      return core::Status::InvalidArgument;
    if (!block.constant) {
      layout->push_back({&block, *active_size});
      *active_size += block.tangent_size;
    }
  }
  if (*active_size == 0 || problem.residual_blocks.empty())
    return core::Status::InvalidArgument;
  return core::Status::Ok;
}

core::Status evaluate_problem(
    SolverProblem &problem,
    const std::unordered_map<ParameterBlockId, std::size_t> &index_by_id,
    const std::vector<LayoutEntry> &layout, std::size_t active_size,
    bool with_jacobian, Evaluation *out) {
  std::unordered_map<ParameterBlockId, std::size_t> active_offset;
  for (const auto &e : layout)
    active_offset.emplace(e.block->id, e.offset);
  std::size_t total_rows = 0;
  for (const auto &rb : problem.residual_blocks) {
    if (!rb.function || rb.function->residual_size() == 0)
      return core::Status::InvalidArgument;
    total_rows += rb.function->residual_size();
  }
  out->rows = total_rows;
  out->cols = active_size;
  out->residuals.assign(total_rows, 0.0);
  out->jacobian.assign(with_jacobian ? total_rows * active_size : 0, 0.0);
  out->blocks.clear();
  std::size_t row_base = 0;
  std::vector<ParameterBlockView> views;
  std::vector<std::vector<double>> local_storage;
  std::vector<JacobianView> local_views;
  for (const auto &rb : problem.residual_blocks) {
    const auto ids = rb.function->parameter_blocks();
    views.clear();
    local_views.clear();
    if (with_jacobian)
      local_storage.resize(ids.size());
    std::size_t local_index = 0;
    views.reserve(ids.size());
    local_storage.reserve(ids.size());
    local_views.reserve(ids.size());
    for (const auto id : ids) {
      const auto it = index_by_id.find(id);
      if (it == index_by_id.end())
        return core::Status::InvalidArgument;
      const auto &p = problem.parameter_blocks[it->second];
      views.push_back({p.id, p.values, p.tangent_size, p.manifold});
      if (with_jacobian) {
        auto &storage = local_storage[local_index++];
        storage.assign(rb.function->residual_size() * p.tangent_size, 0.0);
        local_views.push_back(
            {storage, rb.function->residual_size(), p.tangent_size});
      }
    }
    std::span<double> residual_span(out->residuals.data() + row_base,
                                    rb.function->residual_size());
    const auto status =
        rb.function->evaluate(views, residual_span, local_views);
    if (status != core::Status::Ok)
      return status;
    for (double r : residual_span)
      if (!std::isfinite(r))
        return core::Status::InvalidState;
    if (with_jacobian) {
      Evaluation::RowBlock block{
          row_base, row_base + rb.function->residual_size(), {}};
      for (std::size_t pi = 0; pi < ids.size(); ++pi) {
        const auto off = active_offset.find(ids[pi]);
        if (off != active_offset.end())
          for (std::size_t c = 0; c < local_views[pi].cols; ++c)
            block.columns.push_back(off->second + c);
      }
      std::sort(block.columns.begin(), block.columns.end());
      block.columns.erase(
          std::unique(block.columns.begin(), block.columns.end()),
          block.columns.end());
      out->blocks.push_back(std::move(block));
      for (std::size_t pi = 0; pi < ids.size(); ++pi) {
        const auto off = active_offset.find(ids[pi]);
        if (off == active_offset.end())
          continue;
        const auto &j = local_views[pi];
        for (std::size_t r = 0; r < j.rows; ++r) {
          for (std::size_t c = 0; c < j.cols; ++c) {
            const double value = j(r, c);
            if (!std::isfinite(value))
              return core::Status::InvalidState;
            out->jacobian[(row_base + r) * active_size + off->second + c] =
                value;
          }
        }
      }
    }
    row_base += rb.function->residual_size();
  }
  out->cost = 0.0;
  for (double r : out->residuals)
    out->cost += 0.5 * r * r;
  return std::isfinite(out->cost) ? core::Status::Ok
                                  : core::Status::InvalidState;
}

void normal_equations(const Evaluation &eval, std::vector<double> *h,
                      std::vector<double> *g, double *gradient_max,
                      double *jacobian_norm) {
  const auto n = eval.cols;
  h->assign(n * n, 0.0);
  g->assign(n, 0.0);
  *gradient_max = 0.0;
  *jacobian_norm = 0.0;
  std::vector<std::size_t> nonzero;
  nonzero.reserve(n);
  for (const auto &block : eval.blocks)
    for (std::size_t r = block.begin; r < block.end; ++r) {
      nonzero.clear();
      for (const auto c : block.columns) {
        const double jc = eval.jacobian[r * n + c];
        if (jc == 0)
          continue;
        nonzero.push_back(c);
        (*g)[c] += jc * eval.residuals[r];
        *jacobian_norm += jc * jc;
        for (const auto k : nonzero)
          (*h)[c * n + k] += jc * eval.jacobian[r * n + k];
      }
    }
  for (std::size_t c = 0; c < n; ++c) {
    *gradient_max = std::max(*gradient_max, std::abs((*g)[c]));
    for (std::size_t k = 0; k < c; ++k)
      (*h)[k * n + c] = (*h)[c * n + k];
  }
}

// Bundle adjustment 的独立位姿块仅经全局内参耦合，先消去小块再解全局系统；
// 其他残差图保留通用对称求解器。
bool solve_arrowhead(const std::vector<double> &h,
                     const std::vector<double> &rhs,
                     const std::vector<LayoutEntry> &layout,
                     const numeric::LinearAlgebraBackend &backend,
                     std::vector<double> *solution) {
  const auto n = rhs.size();
  std::size_t global = n, poses = 0;
  for (const auto &e : layout) {
    if (e.block->manifold == ManifoldKind::PoseSE3 &&
        e.block->tangent_size == 6) {
      global = std::min(global, e.offset);
      ++poses;
    } else if (poses)
      return false;
  }
  if (poses < 2 || global == 0 || global > 16 || n != global + 6 * poses)
    return false;
  for (std::size_t p = 0; p < poses; ++p)
    for (std::size_t q = 0; q < p; ++q)
      for (std::size_t i = 0; i < 6; ++i)
        for (std::size_t j = 0; j < 6; ++j)
          if (h[(global + 6 * p + i) * n + global + 6 * q + j] != 0)
            return false;
  std::vector<double> schur(global * global),
      g(rhs.begin(), rhs.begin() + global);
  for (std::size_t i = 0; i < global; ++i)
    for (std::size_t j = 0; j < global; ++j)
      schur[i * global + j] = h[i * n + j];
  std::vector<double> eliminated(poses * 6 * (global + 1));
  for (std::size_t p = 0; p < poses; ++p) {
    const auto start = global + 6 * p;
    double l[6][6]{};
    for (std::size_t i = 0; i < 6; ++i)
      for (std::size_t j = 0; j <= i; ++j) {
        double v = h[(start + i) * n + start + j];
        for (std::size_t k = 0; k < j; ++k)
          v -= l[i][k] * l[j][k];
        if (i == j) {
          if (!(v > 0) || !std::isfinite(v))
            return false;
          l[i][j] = std::sqrt(v);
        } else
          l[i][j] = v / l[j][j];
      }
    auto *inverse = eliminated.data() + p * 6 * (global + 1);
    for (std::size_t c = 0; c <= global; ++c) {
      auto *x = inverse + c * 6;
      for (std::size_t i = 0; i < 6; ++i) {
        double v = c == global ? rhs[start + i] : h[(start + i) * n + c];
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
    }
    for (std::size_t i = 0; i < global; ++i) {
      for (std::size_t k = 0; k < 6; ++k)
        g[i] -= h[i * n + start + k] * inverse[global * 6 + k];
      for (std::size_t j = 0; j < global; ++j)
        for (std::size_t k = 0; k < 6; ++k)
          schur[i * global + j] -= h[i * n + start + k] * inverse[j * 6 + k];
    }
  }
  std::vector<double> x;
  if (!backend.solve_symmetric(schur, g, global, &x))
    return false;
  solution->assign(n, 0);
  std::copy(x.begin(), x.end(), solution->begin());
  for (std::size_t p = 0; p < poses; ++p) {
    const auto *inv = eliminated.data() + p * 6 * (global + 1);
    for (std::size_t k = 0; k < 6; ++k) {
      double v = inv[global * 6 + k];
      for (std::size_t c = 0; c < global; ++c)
        v -= inv[c * 6 + k] * x[c];
      (*solution)[global + 6 * p + k] = v;
    }
  }
  return std::all_of(solution->begin(), solution->end(),
                     [](double v) { return std::isfinite(v); });
}

std::vector<std::vector<double>> snapshot(const SolverProblem &problem) {
  std::vector<std::vector<double>> s;
  s.reserve(problem.parameter_blocks.size());
  for (const auto &p : problem.parameter_blocks)
    s.push_back(p.values);
  return s;
}

void restore(SolverProblem &problem,
             const std::vector<std::vector<double>> &s) {
  for (std::size_t i = 0; i < problem.parameter_blocks.size(); ++i)
    problem.parameter_blocks[i].values = s[i];
}

bool same_values(const SolverProblem &problem,
                 const std::vector<std::vector<double>> &s) {
  for (std::size_t i = 0; i < problem.parameter_blocks.size(); ++i) {
    if (problem.parameter_blocks[i].values != s[i])
      return false;
  }
  return true;
}

} // namespace

core::Status BuiltinLmBackend::solve(SolverProblem &problem,
                                     const SolverOptions &options,
                                     SolverSummary *summary_ptr) {
  SolverSummary local;
  SolverSummary &summary = summary_ptr ? *summary_ptr : local;
  summary = {};
  summary.parameter_blocks = problem.parameter_blocks.size();
  summary.residual_blocks = problem.residual_blocks.size();
  if (options.max_iterations == 0 || options.initial_damping <= 0.0 ||
      options.function_tolerance < 0.0 || options.gradient_tolerance < 0.0 ||
      options.parameter_tolerance < 0.0) {
    summary.termination = SolverTermination::InvalidProblem;
    summary.message = "invalid solver options";
    return core::Status::InvalidArgument;
  }

  std::vector<LayoutEntry> layout;
  std::unordered_map<ParameterBlockId, std::size_t> index_by_id;
  std::size_t n = 0;
  if (build_layout(problem, &layout, &index_by_id, &n) != core::Status::Ok) {
    summary.termination = SolverTermination::InvalidProblem;
    summary.message = "invalid parameter/residual layout";
    return core::Status::InvalidArgument;
  }
  summary.effective_parameters = n;

  Evaluation current;
  auto status =
      evaluate_problem(problem, index_by_id, layout, n, true, &current);
  if (status != core::Status::Ok) {
    summary.termination = SolverTermination::InvalidProblem;
    summary.message = "initial residual evaluation failed";
    return status;
  }
  summary.residuals = current.rows;
  summary.initial_cost = current.cost;
  summary.final_cost = current.cost;
  double lambda = options.initial_damping;
  auto linear_backend = numeric::make_backend(options.linear_backend);

  std::vector<double> base_h, base_g;
  double base_grad = 0, base_jnorm = 0;
  bool changed = true;
  for (std::size_t iter = 0; iter < options.max_iterations; ++iter) {
    summary.iterations = iter + 1;
    std::vector<double> h, g;
    double grad = 0.0;
    double jnorm = 0.0;
    if (changed) {
      normal_equations(current, &base_h, &base_g, &base_grad, &base_jnorm);
      changed = false;
    }
    h = base_h;
    g = base_g;
    grad = base_grad;
    jnorm = base_jnorm;
    summary.gradient_max_norm = grad;
    if (jnorm < 1e-28 && current.cost > options.function_tolerance) {
      summary.termination = SolverTermination::NumericalFailure;
      summary.message = "Jacobian has no observable direction";
      return core::Status::InvalidState;
    }
    if (grad <= options.gradient_tolerance) {
      summary.termination = SolverTermination::Converged;
      summary.message = "gradient tolerance reached";
      return core::Status::Ok;
    }
    for (std::size_t i = 0; i < n; ++i) {
      h[i * n + i] += lambda * std::max(1.0, h[i * n + i]);
      g[i] = -g[i];
    }
    std::vector<double> delta;
    if (!solve_arrowhead(h, g, layout, *linear_backend, &delta) &&
        !linear_backend->solve_symmetric(h, g, n, &delta)) {
      summary.termination = SolverTermination::NumericalFailure;
      summary.message = "damped normal equation solve failed";
      return core::Status::InvalidState;
    }
    double delta_max = 0.0;
    for (double d : delta)
      delta_max = std::max(delta_max, std::abs(d));
    const auto before = snapshot(problem);
    for (const auto &e : layout) {
      const auto block_delta = std::span<const double>(delta.data() + e.offset,
                                                       e.block->tangent_size);
      status = apply_local_delta(*e.block, block_delta);
      if (status != core::Status::Ok) {
        restore(problem, before);
        summary.termination = SolverTermination::NumericalFailure;
        summary.message = "manifold update failed";
        return status;
      }
    }
    if (same_values(problem, before) ||
        delta_max <= options.parameter_tolerance) {
      summary.termination = SolverTermination::Converged;
      summary.message = "parameter tolerance or active bound reached";
      summary.final_cost = current.cost;
      return core::Status::Ok;
    }

    Evaluation candidate;
    status =
        evaluate_problem(problem, index_by_id, layout, n, false, &candidate);
    if (status != core::Status::Ok) {
      restore(problem, before);
      lambda *= 10.0;
      ++summary.unsuccessful_steps;
      continue;
    }
    if (candidate.cost < current.cost) {
      const double improvement = current.cost - candidate.cost;
      ++summary.successful_steps;
      lambda = std::max(lambda / 3.0, 1e-18);
      status =
          evaluate_problem(problem, index_by_id, layout, n, true, &current);
      if (status != core::Status::Ok) {
        restore(problem, before);
        summary.termination = SolverTermination::NumericalFailure;
        summary.message = "accepted-step Jacobian evaluation failed";
        return status;
      }
      changed = true;
      summary.final_cost = current.cost;
      if (improvement <=
          options.function_tolerance * std::max(1.0, current.cost)) {
        summary.termination = SolverTermination::Converged;
        summary.message = "function tolerance reached";
        return core::Status::Ok;
      }
    } else {
      restore(problem, before);
      lambda = std::min(lambda * 10.0, 1e30);
      ++summary.unsuccessful_steps;
      if (!std::isfinite(lambda) || lambda >= 1e30) {
        summary.termination = SolverTermination::NumericalFailure;
        summary.message = "LM damping overflow";
        return core::Status::InvalidState;
      }
    }
  }
  summary.termination = SolverTermination::MaxIterations;
  summary.message = "maximum iterations reached";
  return core::Status::InvalidState;
}

std::unique_ptr<CalibrationOptimizerBackend>
make_optimizer_backend(OptimizerBackendKind kind) {
  switch (kind) {
  case OptimizerBackendKind::BuiltinLm:
    return std::make_unique<BuiltinLmBackend>();
  }
  return nullptr;
}

} // namespace mv::calibration::solver
