#include <mvkit/c/mv_image_processing.h>
#include <imageproc/operators.hpp>
#include "error_state.hpp"
#include "structured_error.hpp"
#include "opaque.hpp"
#include "regions_cache.hpp"
#include "tuple_input.hpp"

extern "C" ::MvStatus mv_watersheds_marker(::MvObject image,::MvObject markers,::MvObject* basins) {
  return mv::c_api::translate([&] {
    using mv::c_api::Exception;using mv::core::Status;using mv::core::ErrorCode;
    constexpr auto op="watersheds_marker";
    if(!image||!markers||!basins)throw Exception(Status::InvalidArgument,ErrorCode::NullArgument,op,"Image/Markers/Basins",-1,"input handles and output required");
    image->value.require_kind(mv::core::IconicKind::Image,op);
    markers->value.require_kind(mv::core::IconicKind::Region,op);
    if(image->value.empty()||markers->value.empty()){*basins=mv::c_api::make_regions(mv::core::Regions{});return;}
    if(image->value.size()!=1)throw Exception(Status::InvalidArgument,ErrorCode::InvalidCardinality,op,"Image",-1,"exactly one image required with nonempty markers");
    const auto first=mv::imageproc::access_channel(image->value.at(0).as_image(),1);
    const auto type=first.pixel_type();
    if(type!=mv::core::PixelType::UInt8&&type!=mv::core::PixelType::UInt16&&type!=mv::core::PixelType::Float32)
      throw Exception(Status::TypeMismatch,ErrorCode::InvalidType,op,"Image",-1,"byte, uint2 or real first channel required");
    auto result=mv::imageproc::watersheds_marker(mv::core::make_image_tuple({first}),mv::c_api::regions_value(markers,op,"Markers"),8);
    *basins=mv::c_api::make_regions(std::move(result));
  });
}
