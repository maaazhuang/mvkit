#pragma once
// 私有所有权适配，不安装、不从 DLL 导出。
#include <mvkit/cpp/mv_iconic.hpp>
#include <utility>
namespace mv { namespace detail {
struct MvHandleAccess {
  static MvHandle Adopt(::MvHandle h) noexcept {return MvHandle(h,MvHandle::AdoptTag{});}
};
struct MvTupleAccess {
  static MvTuple Adopt(::MvTuple h) noexcept {MvTuple value;value.tuple_=h;return value;}
  static void Commit(MvTuple* out,::MvTuple h) noexcept {
    auto value=Adopt(h);
    out->Swap(value);
  }
};
struct MvObjectAccess {
  static MvObject Adopt(::MvObject h) noexcept {return MvObject(h,MvObject::AdoptTag{});}
  static void Commit(MvObject* out,MvObject value) {*out=std::move(value);}
  static void CheckOutput(const MvObject* out,const MvObject& value) {out->CheckKind(value.NativeHandle());}
};
} }

#include <cstring>
#include <mvkit/cpp/mvkit.hpp>

namespace mv { namespace cpp_api {
inline void check_status(::MvStatus status, const char* op) {
  if (status == MV_STATUS_OK) return;
  ::MvErrorInfo info{};
  if (mv_get_last_error_info(&info) == MV_STATUS_OK && info.status == status)
    throw MvException(from_native(status), static_cast<MvErrorCode>(info.code),
        info.operation.size ? MvStringView{info.operation.data,info.operation.size} : MvStringView{op,std::strlen(op)},
        {info.parameter.data,info.parameter.size}, info.parameter_index,
        {info.message.data,info.message.size});
  throw MvException(from_native(status), default_error_code(from_native(status)),
      {op,std::strlen(op)}, {}, -1, "native call failed");
}
inline void require_output(const void* output, const char* op) {
  if (!output) throw MvException(MvStatus::InvalidArgument, MvErrorCode::NullArgument,
      {op,std::strlen(op)}, "output", -1, "output must not be null");
}
} }

#include <array>
namespace mv { namespace cpp_api {
template<std::size_t N> class TupleOutputs {
 public:
  TupleOutputs(const char* op,const std::array<MvTuple*,N>& destinations):destinations_(destinations) {
    for(std::size_t i=0;i<N;++i) {
      require_output(destinations[i],op);
      for(std::size_t j=0;j<i;++j)if(destinations[i]==destinations[j])
        throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidValue,{op,std::strlen(op)},"Output",-1,"output pointers must be distinct");
    }
  }
  TupleOutputs(const TupleOutputs&)=delete;
  ~TupleOutputs(){for(auto value:values_)if(value)(void)mv_destroy_tuple(value);}
  ::MvTuple* Out(std::size_t i) noexcept { return &values_[i]; }
  void Commit() noexcept {
    // C 端已完成整批构造；接管、Swap 及旧值析构均 noexcept，无需再构造 N 个中转元组。
    for(std::size_t i=0;i<N;++i)
      detail::MvTupleAccess::Commit(destinations_[i],std::exchange(values_[i],nullptr));
  }
 private:
  std::array<MvTuple*,N> destinations_;
  std::array<::MvTuple,N> values_{};
};
} }
