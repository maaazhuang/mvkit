#pragma once
#include <c_api/storage_access.hpp>
#include "control_types.hpp"
#include "feature_publication.hpp"
#include "image_types.hpp"
#include "extension_types.hpp"
#include <atomic>
#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include <core/dict.hpp>
#include <core/handle.hpp>
#include <core/image.hpp>
#include <core/iconic_tuple.hpp>
#include <core/regions.hpp>
#include <core/tuple.hpp>
#include <core/xld.hpp>
#include <mvkit/c/mv_region.h>
#include <mvkit/c/mv_image.h>
#include <mvkit/c/mv_xld.h>
#include <shape_matching/model.hpp>
#include <ncc_matching/model.hpp>
#include <calibration/model.hpp>
#include <reconstruction/camera_setup.hpp>
#include <reconstruction/object_model3d.hpp>
#include <reconstruction/stereo/rectification.hpp>
#include <reconstruction/structured_light/model.hpp>
#include <runtime/memory/memory_resource.hpp>
struct MvHandleImpl{std::atomic<std::size_t>refs{1};mv::core::Handle value;};
// get_h 的借用包装器由源 Tuple 拥有；C++ 可以在私有边界保留引用。
struct MvTupleHandleCache {
  std::mutex mutex;
  std::vector<std::pair<std::size_t,MvHandleImpl*>> handles;
  ~MvTupleHandleCache() {
    for(const auto& entry:handles) {
      if(entry.second->refs.fetch_sub(1,std::memory_order_acq_rel)==1) delete entry.second;
    }
  }
};
struct MvTupleImpl {
  std::atomic<std::size_t> refs{1};
  mv::core::Tuple value;
  // refs==0 表示私有只读借用转换存储。
  ::MvTuple borrowed_handle=nullptr;
  const ::MvValue* borrowed_values=nullptr;
  std::size_t borrowed_count=0;
  std::atomic<MvTupleHandleCache*> handle_cache{nullptr};
  explicit MvTupleImpl(std::size_t references=1) noexcept : refs(references) {}
  explicit MvTupleImpl(const mv::core::Tuple& initial) : value(initial) {}
  explicit MvTupleImpl(mv::core::Tuple&& initial) : value(std::move(initial)) {}
  ~MvTupleImpl() { delete handle_cache.load(); }
  void discard_handle_cache() noexcept { delete handle_cache.exchange(nullptr); }
};
struct MvDictImpl{ std::atomic<bool> valid{true};std::atomic<std::size_t>refs{1};mutable std::shared_mutex mutex;mv::core::Dict value;};
struct MVRegionRunDeleter {
  std::pmr::memory_resource* memory=nullptr;std::size_t count=0;
  void operator()(::MvRegionRun* values) const noexcept {
    if(values) {
      if(memory)memory->deallocate(values,count*sizeof(::MvRegionRun),alignof(::MvRegionRun));
      else delete[] values;
    }
  }
};
struct MVRegionRunExport {std::once_flag once;std::unique_ptr<::MvRegionRun[],MVRegionRunDeleter> values;std::size_t count=0;};
struct MVXldPointExport {
  std::atomic<bool> ready{false};
  std::vector<::MvPoint2d> points;
};
struct MvIconicExportCache : mv::c_api::IconicFeaturePublication {
  std::once_flag run_cache_once,centroid_cache_once,rectangle2_cache_once,circle_cache_once;
  std::vector<MVRegionRunExport> run_cache;
  mv::core::Tuple cached_area,cached_row,cached_column,cached_real_area;
  std::array<mv::core::Tuple,5> cached_rectangle2;
  std::unique_ptr<std::array<mv::core::Tuple,3>> cached_circle;
  std::once_flag circularity_once,eccentricity_once;
  mv::core::Tuple cached_circularity;
  std::array<mv::core::Tuple,3> cached_eccentricity;
  std::once_flag orientation_once;
  mv::core::Tuple cached_orientation;
  std::atomic<bool> point_cache_ready{false};
  std::mutex point_cache_mutex;
  std::vector<MVXldPointExport> point_cache;
};
/** 全部 C iconic 输入共享一种存储及生命周期；导出/测量缓存按需分配，纯图像句柄保持紧凑。 */
struct MvObjectImpl {
  std::atomic<std::size_t> refs{1};
  mv::core::IconicTuple value;
  std::atomic<MvIconicExportCache*> export_cache{nullptr};
  ~MvObjectImpl() { delete export_cache.load(std::memory_order_relaxed); }
  MvIconicExportCache& ExportCache() {
    auto* existing=export_cache.load(std::memory_order_acquire);
    if(existing)return *existing;
    auto created=std::make_unique<MvIconicExportCache>();
    if(export_cache.compare_exchange_strong(existing,created.get(),
        std::memory_order_acq_rel,std::memory_order_acquire))return *created.release();
    return *existing;
  }
};



namespace mv::c_api {struct ModelSnapshot;}
struct MvShapeModelImpl{ std::atomic<bool> valid{true};std::atomic<std::size_t>refs{1};mv::shape_matching::ShapeModel value;
  std::mutex snapshot_mutex;std::shared_ptr<const mv::c_api::ModelSnapshot> snapshot;};
struct MvNCCModelImpl{ std::atomic<bool> valid{true};std::atomic<std::size_t>refs{1};mv::ncc_matching::Model value;
  std::mutex snapshot_mutex;std::shared_ptr<const mv::c_api::ModelSnapshot> snapshot;};

struct MvCalibDataImpl{ std::atomic<bool> valid{true};std::atomic<std::size_t>refs{1};std::unique_ptr<mv::calibration::CalibrationDataModel> value;};

namespace mv::c_api { struct ObjectModelCoordinateColumns; }
struct MvObjectModel3DImpl{ std::atomic<bool> valid{true};
  std::atomic<std::size_t> refs{1};
  mv::reconstruction::ObjectModel3DData value;
  mutable std::shared_mutex nearest_mutex;
  std::shared_ptr<const mv::reconstruction::NeighborhoodIndex> nearest_index;
  std::once_flag coordinate_once;
  std::shared_ptr<mv::c_api::ObjectModelCoordinateColumns> coordinate_columns;
};
struct MvCameraSetupModelImpl{ std::atomic<bool> valid{true};std::atomic<std::size_t>refs{1};std::unique_ptr<mv::reconstruction::CameraSetupModel> value;};
struct MvStereoRectificationImpl{ std::atomic<bool> valid{true};std::atomic<std::size_t>refs{1};std::unique_ptr<mv::reconstruction::StereoRectification> value;};
struct MvStructuredLightModelImpl{ std::atomic<bool> valid{true};std::atomic<std::size_t>refs{1};std::unique_ptr<mv::reconstruction::StructuredLightModel> value;};
