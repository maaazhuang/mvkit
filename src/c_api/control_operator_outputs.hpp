#pragma once
#include "opaque.hpp"
#include <array>
#include <core/exception.hpp>
namespace mv::c_api {
/** 分配或执行算法前验证全部输出；包括拥有型控制句柄在内的全部元组构造完成后才提交。 */
template<std::size_t N> class ControlOutputs {
 public:
  ControlOutputs(const char* operation,std::array<::MvTuple*,N> destinations):destinations_(destinations) {
    for(std::size_t i=0;i<N;++i) {
      if(!destinations[i])throw core::Exception(core::Status::InvalidArgument,core::ErrorCode::NullArgument,operation,"Output",i,"output must not be null");
      const auto a=reinterpret_cast<std::uintptr_t>(destinations[i]);
      for(std::size_t j=0;j<i;++j) {
        const auto b=reinterpret_cast<std::uintptr_t>(destinations[j]);
        if((a<b?b-a:a-b)<sizeof(::MvTuple))throw core::Exception(core::Status::InvalidArgument,core::ErrorCode::InvalidValue,operation,"Output",i,"output ranges must be distinct");
      }
    }
    for(auto& value:values_)value=std::make_unique<MvTupleImpl>();
  }
  core::Tuple& operator[](std::size_t index) noexcept {return values_[index]->value;}
  void Commit() noexcept {for(std::size_t i=0;i<N;++i)*destinations_[i]=values_[i].release();}
 private:
  std::array<::MvTuple*,N> destinations_;
  std::array<std::unique_ptr<MvTupleImpl>,N> values_;
};
}
