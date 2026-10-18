#pragma once
// C++ DLL 的句柄状态适配，不安装到 SDK。
#include <mvkit/c/mv_handle.h>
extern "C" MV_API MvStatus mv_private_object_get_type(MvHandle object, MvObjectType* type);
extern "C" MV_API bool mv_private_handle_is_valid(MvHandle object) noexcept;

extern "C" MV_API MvStatus mv_private_object_retain(MvHandle object);

extern "C" MV_API MvStatus mv_private_object_release(MvHandle object);
