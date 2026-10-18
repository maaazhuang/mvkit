#pragma once

// C++ 内联 Tuple 存储的私有 DLL 互操作，不安装、不作为算子/API 入口。
// 构造/析构留在拥有 core::Tuple 的 C DLL 中，避免 C++ DLL 引入第二份私有核心运行库。
#include <mvkit/c/mv_api.h>
#include <mvkit/c/mv_value.h>
#include <mvkit/c/mv_tuple.h>
#include <cstddef>

// 内部借用包装器和实际实现共用容量，变更时同步公开 C++ 类的 ABI 预留。
constexpr std::size_t kMvPrivateTupleInputSize = 96;

extern "C" MV_API void mv_private_tuple_input_init(void* storage,MvTuple handle,
                                                 const MvValue* values,std::size_t count) noexcept;
extern "C" MV_API void mv_private_tuple_input_destroy(void* storage) noexcept;

extern "C" MV_API MvStatus mv_private_tuple_append_i(MvTuple tuple, int64_t value);

extern "C" MV_API MvStatus mv_private_tuple_append_d(MvTuple tuple, double value);

extern "C" MV_API MvStatus mv_private_tuple_append_s(MvTuple tuple, MvStringView value);

extern "C" MV_API MvStatus mv_private_tuple_append_h(MvTuple tuple, MvHandle handle);
