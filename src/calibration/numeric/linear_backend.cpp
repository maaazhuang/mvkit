#include <calibration/numeric/linear_backend.hpp>

#include <stdexcept>
#include <algorithm>

namespace mv::calibration::numeric {

std::unique_ptr<LinearAlgebraBackend> make_fallback_backend();
#ifdef MVKIT_USE_EIGEN
std::unique_ptr<LinearAlgebraBackend> make_eigen_backend();
#endif

std::unique_ptr<LinearAlgebraBackend> make_backend(LinearBackendKind kind) {
  switch (kind) {
    case LinearBackendKind::Fallback:
      return make_fallback_backend();
    case LinearBackendKind::Eigen:
#ifdef MVKIT_USE_EIGEN
      return make_eigen_backend();
#else
      throw std::runtime_error("Eigen numerical backend is disabled in this build");
#endif
  }
  throw std::runtime_error("unknown linear algebra backend");
}

bool LinearAlgebraBackend::solve_symmetric_into(std::span<const double> matrix,
                                               std::span<const double> rhs,
                                               std::span<double> solution) const {
  const auto n=rhs.size();
  if(!n || matrix.size()!=n*n || solution.size()!=n)return false;
  std::vector<double> result;
  if(!solve_symmetric({matrix.begin(),matrix.end()},{rhs.begin(),rhs.end()},n,&result) || result.size()!=n)return false;
  std::copy(result.begin(),result.end(),solution.begin());return true;
}

const LinearAlgebraBackend& shared_backend(LinearBackendKind kind) {
  // 求解矩阵和工作区均为调用局部；不可变策略可跨调用/线程共享，后端显式选择。
  if(kind==LinearBackendKind::Fallback) {
    static const auto backend=make_backend(LinearBackendKind::Fallback);return *backend;
  }
  if(kind==LinearBackendKind::Eigen) {
    static const auto backend=make_backend(LinearBackendKind::Eigen);return *backend;
  }
  throw std::runtime_error("unknown linear algebra backend");
}

LinearBackendKind default_backend_kind() noexcept {
#ifdef MVKIT_USE_EIGEN
  return LinearBackendKind::Eigen;
#else
  return LinearBackendKind::Fallback;
#endif
}

bool eigen_backend_available() noexcept {
#ifdef MVKIT_USE_EIGEN
  return true;
#else
  return false;
#endif
}

}  // namespace mv::calibration::numeric
