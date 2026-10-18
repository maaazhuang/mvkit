#pragma once

#include <cstddef>
#include <cstdint>
#include <mvkit/c/mv_geometry.h>
#include <mvkit/c/mv_region.h>
#include <mvkit/c/mv_xld.h>

// 图标适配器的私有缓存视图和存储类型，不安装到 SDK。
typedef struct MvRegionRun { int32_t row; int32_t col_begin; int32_t col_end; } MvRegionRun;
typedef struct MvRectI64 { int64_t row_min; int64_t col_min; int64_t row_max; int64_t col_max; } MvRectI64;
typedef struct MvXldContourView { const MvPoint2d* points; size_t count; uint8_t closed; } MvXldContourView;

typedef int32_t MvXldPointAttribute;
#define MV_XLD_ATTR_RESPONSE ((int32_t)0)
#define MV_XLD_ATTR_EDGE_DIRECTION ((int32_t)1)
#define MV_XLD_ATTR_NORMAL_ANGLE ((int32_t)2)

typedef int32_t MvXldContourApproximation;
#define MV_XLD_CONTOUR_APPROX_UNKNOWN ((int32_t)0)
#define MV_XLD_CONTOUR_APPROX_LINE ((int32_t)1)
#define MV_XLD_CONTOUR_APPROX_ELLIPSE ((int32_t)2)
#define MV_XLD_CONTOUR_APPROX_CIRCLE ((int32_t)3)
