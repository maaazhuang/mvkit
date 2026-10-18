#include <calibration/numeric/linear_backend.hpp>

#ifdef MVKIT_USE_EIGEN
#include <Eigen/Dense>

#include <cmath>
#include <algorithm>
#include <memory>
#include <array>

namespace mv::calibration::numeric {
namespace {

Eigen::MatrixXd matrix_from_row_major(const std::vector<double>& values,
                                      std::size_t rows, std::size_t cols) {
  Eigen::MatrixXd result(static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(cols));
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t c = 0; c < cols; ++c) {
      result(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = values[r * cols + c];
    }
  }
  return result;
}

class EigenBackend final : public LinearAlgebraBackend {
 public:
  bool solve_symmetric(const std::vector<double>& matrix, const std::vector<double>& rhs,
                       std::size_t n, std::vector<double>* solution) const override {
    if (n == 0 || matrix.size() != n * n || rhs.size() != n || solution == nullptr) return false;
    if(n==6) {
      std::array<double,6> values;
      if(!solve_symmetric_into(matrix,rhs,values))return false;
      solution->assign(values.begin(),values.end());return true;
    }
    const auto a = matrix_from_row_major(matrix, n, n);
    Eigen::VectorXd b(static_cast<Eigen::Index>(n));
    for (std::size_t i = 0; i < n; ++i) b(static_cast<Eigen::Index>(i)) = rhs[i];
    Eigen::LDLT<Eigen::MatrixXd> ldlt(a);
    if (ldlt.info() != Eigen::Success) return false;
    const Eigen::VectorXd x = ldlt.solve(b);
    if (ldlt.info() != Eigen::Success || !x.allFinite()) return false;
    solution->resize(n);
    for (std::size_t i = 0; i < n; ++i) (*solution)[i] = x(static_cast<Eigen::Index>(i));
    return true;
  }

  bool solve_symmetric_into(std::span<const double> matrix,std::span<const double> rhs,
                            std::span<double> solution) const override {
    if(rhs.size()!=6)return LinearAlgebraBackend::solve_symmetric_into(matrix,rhs,solution);
    if(matrix.size()!=36 || solution.size()!=6)return false;
      bool repeated=true;
      for(int i=0;i<3;++i)for(int j=0;j<3;++j)
        repeated=repeated && matrix[i*6+j+3]==0 && matrix[(i+3)*6+j]==0 && matrix[i*6+j]==matrix[(i+3)*6+j+3];
      if(repeated) {
        Eigen::Matrix3d a;for(int i=0;i<3;++i)for(int j=0;j<3;++j)a(i,j)=matrix[i*6+j];
        Eigen::LDLT<Eigen::Matrix3d> ldlt(a);
        if(ldlt.info()!=Eigen::Success)return false;
        Eigen::Matrix<double,3,2> b;for(int i=0;i<3;++i){b(i,0)=rhs[i];b(i,1)=rhs[i+3];}
        const Eigen::Matrix<double,3,2> x=ldlt.solve(b);
        if(ldlt.info()!=Eigen::Success || !x.allFinite())return false;
        std::copy_n(x.data(),6,solution.begin());return true;
      }
      using Matrix=Eigen::Matrix<double,6,6>;
      const Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>> a(matrix.data());
      const Eigen::Map<const Eigen::Matrix<double,6,1>> b(rhs.data());
      Eigen::LDLT<Matrix> ldlt(a);
      if(ldlt.info()!=Eigen::Success)return false;
      const Eigen::Matrix<double,6,1> x=ldlt.solve(b);
      if(ldlt.info()!=Eigen::Success || !x.allFinite())return false;
      std::copy_n(x.data(),6,solution.begin());return true;
  }

  bool smallest_right_singular_vector(const std::vector<double>& matrix,
                                       std::size_t rows, std::size_t cols,
                                       std::vector<double>* vector) const override {
    if (rows == 0 || cols == 0 || matrix.size() != rows * cols || vector == nullptr) return false;
    if(cols==9 && rows>=64) {
      // 高 DLT 矩阵使用正交归约；有界 QR 块保留奇异值，不构造 A^T*A 或完整高度副本。
      using Block=Eigen::Matrix<double,Eigen::Dynamic,9,Eigen::ColMajor,137,9>;
      using Reduced=Eigen::Matrix<double,9,9>;
      Reduced reduced=Reduced::Zero();bool have_previous=false;
      for(std::size_t begin=0;begin<rows;) {
        const auto count=std::min<std::size_t>(128,rows-begin);
        const auto offset=have_previous ? 9 : 0;
        Block block(static_cast<Eigen::Index>(count+offset),9);
        if(have_previous)block.topRows(9)=reduced;
        for(std::size_t r=0;r<count;++r)for(int c=0;c<9;++c) {
          const auto value=matrix[(begin+r)*9+c];
          if(!std::isfinite(value))return false;
          block(static_cast<Eigen::Index>(r+offset),c)=value;
        }
        const Eigen::HouseholderQR<Block> qr(block);
        for(int r=0;r<9;++r)for(int c=0;c<9;++c)reduced(r,c)=c>=r ? qr.matrixQR()(r,c) : 0.;
        have_previous=true;begin+=count;
      }
      const Eigen::JacobiSVD<Reduced> svd(reduced,Eigen::ComputeFullV);
      if(svd.info()!=Eigen::Success || !svd.matrixV().allFinite())return false;
      vector->resize(9);for(int i=0;i<9;++i)(*vector)[i]=svd.matrixV()(i,8);
      return true;
    }
    if(cols==9 && rows<64) {
      // 使用同一 Jacobi SVD 和列主元 QR，工作区由调用方局部持有且有界。
      using Small=Eigen::Matrix<double,Eigen::Dynamic,9,Eigen::ColMajor,63,9>;
      Small a(static_cast<Eigen::Index>(rows),9);
      for(std::size_t r=0;r<rows;++r)for(int c=0;c<9;++c)
        a(static_cast<Eigen::Index>(r),c)=matrix[r*9+c];
      const Eigen::JacobiSVD<Small> svd(a,Eigen::ComputeFullV);
      if(svd.info()!=Eigen::Success || !svd.matrixV().allFinite())return false;
      vector->resize(9);for(int i=0;i<9;++i)(*vector)[i]=svd.matrixV()(i,8);
      return true;
    }
    const auto a = matrix_from_row_major(matrix, rows, cols);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeFullV);
    if (svd.info() != Eigen::Success || !svd.matrixV().allFinite()) return false;
    const Eigen::VectorXd v = svd.matrixV().col(static_cast<Eigen::Index>(cols - 1));
    if (!v.allFinite()) return false;
    vector->resize(cols);
    for (std::size_t i = 0; i < cols; ++i) (*vector)[i] = v(static_cast<Eigen::Index>(i));
    return true;
  }

  bool symmetric_eigen(const std::vector<double>& matrix, std::size_t n,
                       std::vector<double>* eigenvalues,
                       std::vector<double>* eigenvectors) const override {
    if (n == 0 || matrix.size() != n * n || eigenvalues == nullptr || eigenvectors == nullptr) return false;
    const auto a = matrix_from_row_major(matrix, n, n);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(a);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
        !solver.eigenvectors().allFinite()) return false;
    eigenvalues->resize(n);
    eigenvectors->resize(n * n);
    for (std::size_t i = 0; i < n; ++i) {
      (*eigenvalues)[i] = solver.eigenvalues()(static_cast<Eigen::Index>(i));
      for (std::size_t r = 0; r < n; ++r) {
        (*eigenvectors)[r * n + i] = solver.eigenvectors()(static_cast<Eigen::Index>(r),
                                                           static_cast<Eigen::Index>(i));
      }
    }
    return true;
  }
};

}  // namespace

std::unique_ptr<LinearAlgebraBackend> make_eigen_backend() {
  return std::make_unique<EigenBackend>();
}

}  // namespace mv::calibration::numeric
#endif
