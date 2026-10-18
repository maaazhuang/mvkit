#pragma once

#include <reconstruction/object_model3d.hpp>

namespace mv::reconstruction {
// 调用方已验证模型及轴号；使用精确闭区间，不扩展一 ULP，保持正式 C 算子语义。
ObjectModel3DData compact_coordinate_points(const ObjectModel3DData&,std::size_t axis,double lower,double upper,
                                           const PointAttributes::Snapshot&);
}
