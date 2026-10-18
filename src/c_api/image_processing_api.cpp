#include <mvkit/c/mv_image_processing.h>

#include <memory>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <string_view>

#include "error_state.hpp"
#include "structured_error.hpp"
#include "iconic_handle.hpp"
#include "opaque.hpp"
#include "regions_cache.hpp"
#include "tuple_input.hpp"
#include "image_controls.hpp"
#include "control_operator_outputs.hpp"
#include <core/exception.hpp>
#include <imageproc/operators.hpp>
#include <imageproc/color_planes.hpp>

namespace {
using mv::c_api::Exception;
using mv::core::Status;

void require_image(::MvObject image,const char* where){if(!image) throw Exception(Status::InvalidArgument,where,"image handle must not be null");image->value.require_kind(mv::core::IconicKind::Image,where);}
void require_regions(::MvObject regions,const char* where){if(!regions) throw Exception(Status::InvalidArgument,where,"regions handle must not be null");regions->value.require_kind(mv::core::IconicKind::Region,where);}
const mv::core::Region& require_single(::MvObject regions,const char* where){require_regions(regions,where);if(regions->value.size()!=1) throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,where,"region",-1,"exactly one region object is required");return regions->value.at(0).as_region();}
void require_output(const void* out,const char* where){if(!out) throw Exception(Status::InvalidArgument,where,"output pointer must not be null");}
const mv::core::Tuple& require_tuple(::MvTuple tuple,const char* where,const char* parameter){if(!tuple)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::NullArgument,where,parameter,-1,"tuple handle must not be null");return tuple->value;}

template<class Kernel>
void binary_control_image(::MvObject left,::MvObject right,::MvObject* output,
    ::MvTuple Mult,::MvTuple Add,bool absolute,const char* op,
    const char* left_name,const char* right_name,const char* output_name,Kernel&& kernel) {
  mv::c_api::require_iconic(left,op,left_name);
  mv::c_api::require_iconic(right,op,right_name);
  mv::c_api::require_output(output,op,output_name);
  left->value.require_kind(mv::core::IconicKind::Image,op);
  right->value.require_kind(mv::core::IconicKind::Image,op);
  // 任一 iconic 输入为空时，HALCON 在检查控制内容前返回零对象；右侧单对象供全部左侧复用。
  if(left->value.empty()||right->value.empty()) {
    auto result=mv::c_api::make_iconic(mv::core::IconicTuple{});
    *output=result.release();return;
  }
  if(right->value.size()!=1&&right->value.size()!=left->value.size())
    throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,
        op,right_name,-1,"right input must contain one object or match the left input count");
  const auto mult=mv::c_api::TupleInput(Mult,op,"Mult").scalar_number();
  if(absolute&&mult<0)mv::c_api::TupleInput(Mult,op,"Mult").fail(mv::core::ErrorCode::InvalidValue,0,"nonnegative multiplier required");
  const auto add=absolute?0:mv::c_api::TupleInput(Add,op,"Add").scalar_number();
  auto result=mv::c_api::make_iconic(kernel(left->value,right->value,mult,add));
  *output=result.release();
}

void image_operand(::MvObject image,const char* op,const char* parameter) {
  mv::c_api::require_iconic(image,op,parameter);
  image->value.require_kind(mv::core::IconicKind::Image,op);
}
void region_operand(::MvObject region,const char* op,const char* parameter) {
  mv::c_api::require_iconic(region,op,parameter);
  region->value.require_kind(mv::core::IconicKind::Region,op);
}
void empty_image(::MvObject* output) {auto result=mv::c_api::make_iconic(mv::core::IconicTuple{});*output=result.release();}
mv::imageproc::Interpolation image_interpolation(const mv::c_api::TupleInput& input) {
  const auto name=mv::c_api::image_string(input);
  if(name=="nearest_neighbor")return mv::imageproc::Interpolation::NearestNeighbor;
  if(name=="bilinear")return mv::imageproc::Interpolation::Bilinear;
  if(name=="constant")return mv::imageproc::Interpolation::Constant;
  if(name=="weighted")return mv::imageproc::Interpolation::Weighted;
  if(name=="bicubic")return mv::imageproc::Interpolation::Bicubic;
  input.fail(mv::core::ErrorCode::InvalidValue,0,"unsupported interpolation");
}

::MvObject make_image(mv::core::Image value){return mv::c_api::make_image_handle(std::move(value)).release();}
::MvObject make_image(mv::core::IconicTuple value){return mv::c_api::make_image_handle(std::move(value),"image processing output").release();}

}

extern "C" ::MvStatus mv_get_domain(::MvObject Image,::MvObject* Domain) {
  return mv::c_api::translate([&] {
    constexpr auto op="get_domain";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(Domain,op,"Domain");
    auto result=mv::c_api::make_iconic(mv::imageproc::get_domain(Image->value));
    *Domain=result.release();
  });
}

namespace {
void require_pair(::MvTuple* first,::MvTuple* second,const char* operation) {
  require_output(first,operation);require_output(second,operation);
  if(first==second) throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,
      operation,"Output",-1,"output pointers must be distinct");
}
void commit_pair(mv::core::Tuple first,mv::core::Tuple second,::MvTuple* out_first,::MvTuple* out_second) {
  auto a=std::make_unique<MvTupleImpl>();auto b=std::make_unique<MvTupleImpl>();
  a->value=std::move(first);b->value=std::move(second);
  *out_first=a.release();*out_second=b.release();
}
void require_at_most_one(const mv::core::IconicTuple& value,const char* op,const char* parameter) {
  if(value.size()>1)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,
      op,parameter,-1,"at most one iconic object required");
}
}

extern "C" ::MvStatus mv_convert_image_type(::MvObject Image,::MvObject* ImageConverted,::MvTuple NewType) {
  return mv::c_api::translate([&] {
    constexpr auto op="convert_image_type";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageConverted,op,"ImageConverted");
    const mv::c_api::TupleInput input(NewType,op,"NewType");
    if(input.size()!=1)input.fail(mv::core::ErrorCode::InvalidCardinality,-1,"exactly one type required");
    const auto name=input.string(0);
    mv::core::PixelType type;
    if(name=="byte")type=mv::core::PixelType::UInt8;
    else if(name=="uint2")type=mv::core::PixelType::UInt16;
    else if(name=="int2")type=mv::core::PixelType::Int16;
    else if(name=="int4")type=mv::core::PixelType::Int32;
    else if(name=="real")type=mv::core::PixelType::Float32;
    else input.fail(mv::core::ErrorCode::InvalidValue,0,"unsupported image type");
    auto result=mv::c_api::make_iconic(mv::imageproc::convert_image_type(Image->value,type));
    *ImageConverted=result.release();
  });
}
extern "C" ::MvStatus mv_access_channel(::MvObject MultiChannelImage,::MvObject* Image,::MvTuple Channel) {
  return mv::c_api::translate([&] {
    constexpr auto op="access_channel";
    mv::c_api::require_iconic(MultiChannelImage,op,"MultiChannelImage");
    mv::c_api::require_output(Image,op,"Image");
    require_at_most_one(MultiChannelImage->value,op,"MultiChannelImage");
    const mv::c_api::TupleInput input(Channel,op,"Channel");
    const auto channel=input.scalar_integer();
    if(channel<1) throw Exception(Status::OutOfRange,mv::core::ErrorCode::IndexOutOfRange,op,"Channel",0,"channel must be positive");
    auto result=mv::c_api::make_iconic(mv::imageproc::access_channel(MultiChannelImage->value,static_cast<std::size_t>(channel)));
    *Image=result.release();
  });
}
extern "C" ::MvStatus mv_compose3(::MvObject Image1,::MvObject Image2,::MvObject Image3,::MvObject* MultiChannelImage) {
  return mv::c_api::translate([&] {
    constexpr auto op="compose3";
    mv::c_api::require_iconic(Image1,op,"Image1");
    mv::c_api::require_iconic(Image2,op,"Image2");
    mv::c_api::require_iconic(Image3,op,"Image3");
    mv::c_api::require_output(MultiChannelImage,op,"MultiChannelImage");
    if(Image1->value.size()!=Image2->value.size()||Image1->value.size()!=Image3->value.size())
      throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,op,"Image2",-1,"image arrays must have equal lengths");
    auto result=mv::c_api::make_iconic(mv::imageproc::compose3(Image1->value,Image2->value,Image3->value));
    *MultiChannelImage=result.release();
  });
}
extern "C" ::MvStatus mv_decompose3(::MvObject MultiChannelImage,::MvObject* Image1,::MvObject* Image2,::MvObject* Image3) {
  return mv::c_api::translate([&] {
    constexpr auto op="decompose3";
    mv::c_api::require_iconic(MultiChannelImage,op,"MultiChannelImage");
    mv::c_api::require_output(Image1,op,"Image1");
    mv::c_api::require_output(Image2,op,"Image2");
    mv::c_api::require_output(Image3,op,"Image3");
    if(Image1==Image2||Image1==Image3||Image2==Image3)
      throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"Output",-1,"outputs must be distinct");
    MultiChannelImage->value.require_kind(mv::core::IconicKind::Image,op);
    for(const auto& value:MultiChannelImage->value.values())if(value.as_image().channel_count()!=3)
      throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,op,"MultiChannelImage",-1,"exactly three channels required");
    if(MultiChannelImage->value.size()==1) {
      auto views=MultiChannelImage->value.at(0).as_image().channel_views3();
      auto a=mv::c_api::make_image_handle(std::move(views[0]));
      auto b=mv::c_api::make_image_handle(std::move(views[1]));
      auto c=mv::c_api::make_image_handle(std::move(views[2]));
      *Image1=a.release();*Image2=b.release();*Image3=c.release();
      return;
    }
    auto a=mv::c_api::make_iconic(mv::imageproc::access_channel(MultiChannelImage->value,1));
    auto b=mv::c_api::make_iconic(mv::imageproc::access_channel(MultiChannelImage->value,2));
    auto c=mv::c_api::make_iconic(mv::imageproc::access_channel(MultiChannelImage->value,3));
    *Image1=a.release();*Image2=b.release();*Image3=c.release();
  });
}
extern "C" ::MvStatus mv_intensity(::MvObject Regions,::MvObject Image,::MvTuple* Mean,::MvTuple* Deviation) {
  return mv::c_api::translate([&] {
    constexpr auto op="intensity";
    mv::c_api::require_iconic(Regions,op,"Regions");
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(Mean,op,"Mean");
    mv::c_api::require_output(Deviation,op,"Deviation");
    require_pair(Mean,Deviation,op);
    Regions->value.require_kind(mv::core::IconicKind::Region,op);
    Image->value.require_kind(mv::core::IconicKind::Image,op);
    require_at_most_one(Image->value,op,"Image");
    mv::imageproc::IntensityTuples result;
    if(!Image->value.empty()) {
      const auto full=Image->value.at(0).as_image().full_domain();
      if(full.channel_count()!=1)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,op,"Image",-1,"exactly one channel required");
      for(const auto& value:Regions->value.values()) {
        const auto region=value.as_region().intersect(full.domain());
        const auto stats=region.empty() ? mv::imageproc::IntensityResult{0,0} : mv::imageproc::intensity(region,full);
        result.mean.append(stats.mean);result.deviation.append(stats.deviation);
      }
    }
    commit_pair(std::move(result.mean),std::move(result.deviation),Mean,Deviation);
  });
}
extern "C" ::MvStatus mv_gray_histo(::MvObject Region,::MvObject Image,::MvTuple* AbsoluteHisto,::MvTuple* RelativeHisto) {
  return mv::c_api::translate([&] {
    constexpr auto op="gray_histo";
    mv::c_api::require_iconic(Region,op,"Region");
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(AbsoluteHisto,op,"AbsoluteHisto");
    mv::c_api::require_output(RelativeHisto,op,"RelativeHisto");
    require_pair(AbsoluteHisto,RelativeHisto,op);
    Region->value.require_kind(mv::core::IconicKind::Region,op);
    Image->value.require_kind(mv::core::IconicKind::Image,op);
    require_at_most_one(Region->value,op,"Region");
    require_at_most_one(Image->value,op,"Image");
    mv::imageproc::HistogramResult result;
    if(!Region->value.empty()&&!Image->value.empty())
      result=mv::imageproc::gray_histo_standard(Region->value.at(0).as_region(),Image->value.at(0).as_image());
    commit_pair(std::move(result.absolute),std::move(result.relative),AbsoluteHisto,RelativeHisto);
  });
}
extern "C" ::MvStatus mv_binary_threshold(::MvObject Image,::MvObject* Region,
    ::MvTuple Method,::MvTuple LightDark,::MvTuple* UsedThreshold) {
  return mv::c_api::translate([&] {
    constexpr auto op="binary_threshold";image_operand(Image,op,"Image");
    mv::c_api::require_output(Region,op,"Region");mv::c_api::require_output(UsedThreshold,op,"UsedThreshold");
    const auto a=reinterpret_cast<std::uintptr_t>(Region),b=reinterpret_cast<std::uintptr_t>(UsedThreshold);
    if((a<b?b-a:a-b)<sizeof(::MvObject))throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"Output",-1,"output byte ranges must be distinct");
    mv::c_api::ControlOutputs<1> controls(op,{{UsedThreshold}});
    if(Image->value.empty()){auto result=mv::c_api::make_iconic(mv::core::IconicTuple{});*Region=result.release();controls.Commit();return;}
    const mv::c_api::TupleInput method(Method,op,"Method"),polarity(LightDark,op,"LightDark");
    const auto name=mv::c_api::image_string(method),mode=mv::c_api::image_string(polarity);
    if(name!="max_separability")throw Exception(Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"Method",0,"only max_separability is implemented");
    if(mode!="light"&&mode!="dark")polarity.fail(mv::core::ErrorCode::InvalidValue,0,"expected light or dark");
    const auto first_channels=mv::imageproc::access_channel(Image->value,1);
    auto result=mv::imageproc::binary_threshold(first_channels,mode=="light"?mv::imageproc::ThresholdPolarity::Light:mv::imageproc::ThresholdPolarity::Dark);
    auto region=mv::c_api::make_iconic(std::move(result.regions));controls[0]=std::move(result.thresholds);
    *Region=region.release();controls.Commit();
  });
}
extern "C" ::MvStatus mv_reduce_domain(::MvObject Image,::MvObject Region,::MvObject* ImageReduced) {
  return mv::c_api::translate([&] {
    constexpr auto op="reduce_domain";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_iconic(Region,op,"Region");
    mv::c_api::require_output(ImageReduced,op,"ImageReduced");
    Region->value.require_kind(mv::core::IconicKind::Region,op);
    Image->value.require_kind(mv::core::IconicKind::Image,op);
    // HALCON 将首个区域应用于每个图像对象。
    const auto regions=Region->value.empty() ? mv::core::Regions{} : mv::core::Regions{mv::core::IconicTuple{std::vector<mv::core::IconicValue>{Region->value.at(0)}}};
    auto result=mv::c_api::make_iconic(mv::imageproc::reduce_domain(Image->value,regions));
    *ImageReduced=result.release();
  });
}
extern "C" ::MvStatus mv_full_domain(::MvObject Image,::MvObject* ImageFull) {
  return mv::c_api::translate([&] {
    constexpr auto op="full_domain";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageFull,op,"ImageFull");
    auto result=mv::c_api::make_iconic(mv::imageproc::full_domain(Image->value));
    *ImageFull=result.release();
  });
}
extern "C" ::MvStatus mv_threshold(::MvObject image,::MvObject* region,
    ::MvTuple min_gray,::MvTuple max_gray) {
  return mv::c_api::translate([&] {
    mv::c_api::require_iconic(image,"threshold","Image");
    mv::c_api::require_output(region,"threshold","Region");
    const mv::c_api::TupleInput minimum(min_gray,"threshold","MinGray"),
        maximum(max_gray,"threshold","MaxGray");
    auto result=mv::c_api::make_iconic(mv::imageproc::threshold(image->value,
        minimum.materialize(),maximum.materialize()));
    *region=result.release();
  });
}
extern "C" ::MvStatus mv_dyn_threshold(::MvObject OrigImage,::MvObject ThresholdImage,::MvObject* RegionDynThresh,::MvTuple Offset,::MvTuple LightDark) {
  return mv::c_api::translate([&]{
    constexpr auto op="dyn_threshold";
    image_operand(OrigImage,op,"OrigImage");image_operand(ThresholdImage,op,"ThresholdImage");
    mv::c_api::require_output(RegionDynThresh,op,"RegionDynThresh");
    if(OrigImage->value.empty()||ThresholdImage->value.empty()){empty_image(RegionDynThresh);return;}
    if(OrigImage->value.size()!=ThresholdImage->value.size())throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,op,"ThresholdImage",-1,"nonempty image arrays must have equal lengths");
    const auto offset=mv::c_api::TupleInput(Offset,op,"Offset").scalar_number();
    const mv::c_api::TupleInput input(LightDark,op,"LightDark");const auto name=mv::c_api::image_string(input);
    mv::imageproc::DynThresholdMode mode;
    if(name=="light")mode=mv::imageproc::DynThresholdMode::Light;
    else if(name=="dark")mode=mv::imageproc::DynThresholdMode::Dark;
    else if(name=="equal")mode=mv::imageproc::DynThresholdMode::Equal;
    else if(name=="not_equal")mode=mv::imageproc::DynThresholdMode::NotEqual;
    else input.fail(mv::core::ErrorCode::InvalidValue,0,"unsupported threshold mode");
    std::vector<mv::core::IconicValue> regions;regions.reserve(OrigImage->value.size());
    for(std::size_t i=0;i<OrigImage->value.size();++i){const auto& a=OrigImage->value.at(i).as_image();const auto& b=ThresholdImage->value.at(i).as_image();
      const bool integral=a.pixel_type()!=mv::core::PixelType::Float32&&b.pixel_type()!=mv::core::PixelType::Float32;
      regions.push_back(mv::core::IconicValue::from_region(mv::imageproc::dyn_threshold(a,b,integral?std::floor(offset+0.5):offset,mode)));}
    auto result=mv::c_api::make_iconic(mv::core::IconicTuple{std::move(regions)});*RegionDynThresh=result.release();
  });
}
extern "C" ::MvStatus mv_mean_image(::MvObject Image, ::MvObject* ImageMean, ::MvTuple MaskWidth, ::MvTuple MaskHeight) {
  return mv::c_api::translate([&] {
    constexpr auto op="mean_image";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageMean,op,"ImageMean");
    const auto width=mv::c_api::TupleInput(MaskWidth,op,"MaskWidth").scalar_integer();
    if(width<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskWidth",0,"positive integer required");
    const auto height=mv::c_api::TupleInput(MaskHeight,op,"MaskHeight").scalar_integer();
    if(height<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskHeight",0,"positive integer required");
    auto result=mv::c_api::make_iconic(mv::imageproc::mean_image(Image->value,static_cast<std::size_t>(width),static_cast<std::size_t>(height)));
    *ImageMean=result.release();
  });
}
extern "C" ::MvStatus mv_gauss_filter(::MvObject Image, ::MvObject* ImageGauss, ::MvTuple Size) {
  return mv::c_api::translate([&] {
    constexpr auto op="gauss_filter";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageGauss,op,"ImageGauss");
    const auto size=mv::c_api::TupleInput(Size,op,"Size").scalar_integer();
    if(size<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"Size",0,"positive integer required");
    auto result=mv::c_api::make_iconic(mv::imageproc::gauss_filter_standard(Image->value,static_cast<std::size_t>(size)));
    *ImageGauss=result.release();
  });
}
extern "C" ::MvStatus mv_median_image(::MvObject Image,::MvObject* ImageMedian,
    ::MvTuple MaskType,::MvTuple Radius,::MvTuple Margin) {
  return mv::c_api::translate([&] {
    constexpr auto op="median_image";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageMedian,op,"ImageMedian");
    Image->value.require_kind(mv::core::IconicKind::Image,op);
    const mv::c_api::TupleInput mask(MaskType,op,"MaskType"),radius(Radius,op,"Radius"),margin(Margin,op,"Margin");
    if(mask.size()!=1)mask.fail(mv::core::ErrorCode::InvalidCardinality,-1,"exactly one mask type required");
    const auto name=mask.string(0);
    mv::imageproc::MedianMask kind;
    if(name=="square")kind=mv::imageproc::MedianMask::Square;
    else if(name=="circle")kind=mv::imageproc::MedianMask::Circle;
    else mask.fail(mv::core::ErrorCode::UnsupportedValue,0,"unsupported mask type");
    const auto size=radius.scalar_integer();
    if(size<1||size>4095)radius.fail(mv::core::ErrorCode::InvalidValue,0,"radius must be 1..4095");
    for(const auto& object:Image->value.values()) {
      const auto& image=object.as_image();
      if(static_cast<std::uint64_t>(2*size+1)>image.width()||static_cast<std::uint64_t>(2*size+1)>image.height())
        radius.fail(mv::core::ErrorCode::InvalidValue,0,"mask diameter must not exceed image dimensions");
    }
    if(margin.size()!=1)margin.fail(mv::core::ErrorCode::InvalidCardinality,-1,"exactly one margin required");
    mv::imageproc::BorderMode border;
    double constant=0;
    if(margin.at(0).kind()==mv::core::ValueKind::String) {
      const auto mode=margin.string(0);
      if(mode=="mirrored")border=mv::imageproc::BorderMode::Mirrored;
      else if(mode=="continued")border=mv::imageproc::BorderMode::Continued;
      else if(mode=="cyclic")border=mv::imageproc::BorderMode::Cyclic;
      else margin.fail(mv::core::ErrorCode::UnsupportedValue,0,"unsupported margin");
    } else {border=mv::imageproc::BorderMode::Constant;constant=margin.scalar_number();}
    auto result=mv::c_api::make_iconic(mv::imageproc::median_image(Image->value,kind,static_cast<std::size_t>(size),border,constant));
    *ImageMedian=result.release();
  });
}
extern "C" ::MvStatus mv_scale_image(::MvObject Image, ::MvObject* ImageScaled, ::MvTuple Mult, ::MvTuple Add) {
  return mv::c_api::translate([&] {
    constexpr auto op="scale_image";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageScaled,op,"ImageScaled");
    const auto mult=mv::c_api::TupleInput(Mult,op,"Mult").scalar_number();
    const auto add=mv::c_api::TupleInput(Add,op,"Add").scalar_number();
    auto result=mv::c_api::make_iconic(mv::imageproc::scale_image(Image->value,mult,add));
    *ImageScaled=result.release();
  });
}
extern "C" ::MvStatus mv_add_image(::MvObject Image1, ::MvObject Image2, ::MvObject* ImageResult, ::MvTuple Mult, ::MvTuple Add) {
  return mv::c_api::translate([&] {
    binary_control_image(Image1,Image2,ImageResult,Mult,Add,false,"add_image","Image1","Image2","ImageResult",
        [](const auto& a,const auto& b,double mult,double add) { return mv::imageproc::add_image(a,b,mult,add); });
  });
}

extern "C" ::MvStatus mv_sub_image(::MvObject ImageMinuend, ::MvObject ImageSubtrahend, ::MvObject* ImageSub, ::MvTuple Mult, ::MvTuple Add) {
  return mv::c_api::translate([&] {
    binary_control_image(ImageMinuend,ImageSubtrahend,ImageSub,Mult,Add,false,"sub_image","ImageMinuend","ImageSubtrahend","ImageSub",
        [](const auto& a,const auto& b,double mult,double add) { return mv::imageproc::sub_image(a,b,mult,add); });
  });
}

extern "C" ::MvStatus mv_mult_image(::MvObject Image1, ::MvObject Image2, ::MvObject* ImageResult, ::MvTuple Mult, ::MvTuple Add) {
  return mv::c_api::translate([&] {
    binary_control_image(Image1,Image2,ImageResult,Mult,Add,false,"mult_image","Image1","Image2","ImageResult",
        [](const auto& a,const auto& b,double mult,double add) { return mv::imageproc::mult_image(a,b,mult,add); });
  });
}

extern "C" ::MvStatus mv_min_max_gray(::MvObject Regions,::MvObject Image,::MvTuple Percent,::MvTuple* Min,::MvTuple* Max,::MvTuple* Range) {
  return mv::c_api::translate([&]{
    constexpr auto op="min_max_gray";
    region_operand(Regions,op,"Regions");image_operand(Image,op,"Image");
    mv::c_api::ControlOutputs<3> outputs(op,{{Min,Max,Range}});
    if(Regions->value.empty()||Image->value.empty()){outputs.Commit();return;}
    require_at_most_one(Image->value,op,"Image");
    const mv::c_api::TupleInput input(Percent,op,"Percent");const auto percent=input.scalar_number();
    if(percent<0||percent>50)input.fail(mv::core::ErrorCode::InvalidValue,0,"percent must be in [0,50]");
    const auto image=Image->value.at(0).as_image().full_domain();
    for(const auto& value:Regions->value.values()) {
      const auto stats=mv::imageproc::min_max_gray(value.as_region(),image,percent);
      outputs[0].append(stats.min);outputs[1].append(stats.max);outputs[2].append(stats.range);
    }
    outputs.Commit();
  });
}
extern "C" ::MvStatus mv_sobel_amp(::MvObject Image,::MvObject* EdgeAmplitude,::MvTuple FilterType,::MvTuple Size) {
  return mv::c_api::translate([&]{
    constexpr auto op="sobel_amp";image_operand(Image,op,"Image");mv::c_api::require_output(EdgeAmplitude,op,"EdgeAmplitude");
    if(Image->value.empty()){empty_image(EdgeAmplitude);return;}
    const mv::c_api::TupleInput fi(FilterType,op,"FilterType"),si(Size,op,"Size");auto name=mv::c_api::image_string(fi);
    const bool binomial=name.ends_with("_binomial");if(binomial)name.remove_suffix(9);
    if(si.size()!=1&&si.size()!=2)si.fail(mv::core::ErrorCode::InvalidCardinality,-1,"one size or two directional sizes required");
    const auto first=mv::c_api::image_integer(si),second=si.size()==2?mv::c_api::image_integer(si,1):first;
    for(const auto size:{first,second})if(size<3||size>(binomial?39:13)||(size%2)==0)si.fail(mv::core::ErrorCode::InvalidValue,0,"unsupported Sobel size");
    mv::imageproc::SobelFilter filter;
    if(name=="sum_abs")filter=mv::imageproc::SobelFilter::SumAbs;
    else if(name=="sum_sqrt")filter=mv::imageproc::SobelFilter::SumSqrt;
    else if(name=="x")filter=mv::imageproc::SobelFilter::X;
    else if(name=="y")filter=mv::imageproc::SobelFilter::Y;
    else if(name=="thin_sum_abs"||name=="thin_max_abs")throw Exception(Status::Unsupported,mv::core::ErrorCode::InvalidValue,op,"FilterType",0,"Sobel thinning is not implemented");
    else fi.fail(mv::core::ErrorCode::InvalidValue,0,"unsupported Sobel filter");
    if(first!=second||(binomial&&first!=3))throw Exception(Status::Unsupported,mv::core::ErrorCode::InvalidValue,op,"Size",-1,"anisotropic and larger binomial Sobel kernels are not implemented");
    auto result=mv::c_api::make_iconic(mv::imageproc::sobel_amp(Image->value,filter,static_cast<std::size_t>(first)));*EdgeAmplitude=result.release();
  });
}
extern "C" ::MvStatus mv_crop_rectangle1(::MvObject Image,::MvObject* ImagePart,::MvTuple Row1,::MvTuple Column1,::MvTuple Row2,::MvTuple Column2) {
  return mv::c_api::translate([&]{
    constexpr auto op="crop_rectangle1";image_operand(Image,op,"Image");mv::c_api::require_output(ImagePart,op,"ImagePart");
    if(Image->value.empty()){empty_image(ImagePart);return;}
    const auto r1=mv::c_api::image_scalar_integer(mv::c_api::TupleInput(Row1,op,"Row1"));
    const auto c1=mv::c_api::image_scalar_integer(mv::c_api::TupleInput(Column1,op,"Column1"));
    const auto r2=mv::c_api::image_scalar_integer(mv::c_api::TupleInput(Row2,op,"Row2"));
    const auto c2=mv::c_api::image_scalar_integer(mv::c_api::TupleInput(Column2,op,"Column2"));
    auto result=mv::c_api::make_iconic(mv::imageproc::crop_rectangle1(Image->value,r1,c1,r2,c2));*ImagePart=result.release();
  });
}
extern "C" ::MvStatus mv_mirror_image(::MvObject Image,::MvObject* ImageMirror,::MvTuple Mode) {
  return mv::c_api::translate([&]{
    constexpr auto op="mirror_image";image_operand(Image,op,"Image");mv::c_api::require_output(ImageMirror,op,"ImageMirror");
    const mv::c_api::TupleInput input(Mode,op,"Mode");const auto name=mv::c_api::image_string(input);
    mv::imageproc::MirrorMode mode;
    if(name=="row")mode=mv::imageproc::MirrorMode::Row;
    else if(name=="column")mode=mv::imageproc::MirrorMode::Column;
    else if(name=="diagonal")mode=mv::imageproc::MirrorMode::Diagonal;
    else input.fail(mv::core::ErrorCode::InvalidValue,0,"unsupported mirror mode");
    auto result=mv::c_api::make_iconic(mv::imageproc::mirror_image(Image->value,mode));*ImageMirror=result.release();
  });
}
extern "C" ::MvStatus mv_zoom_image_size(::MvObject Image,::MvObject* ImageZoom,::MvTuple Width,::MvTuple Height,::MvTuple Interpolation) {
  return mv::c_api::translate([&]{
    constexpr auto op="zoom_image_size";image_operand(Image,op,"Image");mv::c_api::require_output(ImageZoom,op,"ImageZoom");
    const mv::c_api::TupleInput wi(Width,op,"Width"),hi(Height,op,"Height");
    const auto w=mv::c_api::image_scalar_integer(wi),h=mv::c_api::image_scalar_integer(hi);
    if(w<1)wi.fail(mv::core::ErrorCode::InvalidValue,0,"positive width required");
    if(h<1)hi.fail(mv::core::ErrorCode::InvalidValue,0,"positive height required");
    const auto mode=image_interpolation(mv::c_api::TupleInput(Interpolation,op,"Interpolation"));
    std::vector<mv::core::Image> images;images.reserve(Image->value.size());
    for(const auto& object:Image->value.values())images.push_back(mv::imageproc::zoom_image_size(object.as_image().full_domain(),static_cast<std::size_t>(w),static_cast<std::size_t>(h),mode));
    auto result=mv::c_api::make_iconic(mv::core::make_image_tuple(std::move(images)));*ImageZoom=result.release();
  });
}
extern "C" ::MvStatus mv_rgb1_to_gray(::MvObject RGBImage,::MvObject* GrayImage) {
  return mv::c_api::translate([&] {
    constexpr auto op="rgb1_to_gray";
    mv::c_api::require_iconic(RGBImage,op,"RGBImage");
    mv::c_api::require_output(GrayImage,op,"GrayImage");
    auto result=mv::c_api::make_iconic(mv::imageproc::rgb1_to_gray(RGBImage->value));
    *GrayImage=result.release();
  });
}

extern "C" ::MvStatus mv_abs_diff_image(::MvObject Image1, ::MvObject Image2, ::MvObject* ImageAbsDiff, ::MvTuple Mult) {
  return mv::c_api::translate([&] {
    binary_control_image(Image1,Image2,ImageAbsDiff,Mult,::MvTuple{},true,"abs_diff_image","Image1","Image2","ImageAbsDiff",
        [](const auto& a,const auto& b,double mult,double add) {  (void)add;return mv::imageproc::abs_diff_image(a,b,mult); });
  });
}

extern "C" ::MvStatus mv_div_image(::MvObject Image1, ::MvObject Image2, ::MvObject* ImageResult, ::MvTuple Mult, ::MvTuple Add) {
  return mv::c_api::translate([&] {
    binary_control_image(Image1,Image2,ImageResult,Mult,Add,false,"div_image","Image1","Image2","ImageResult",
        [](const auto& a,const auto& b,double mult,double add) { return mv::imageproc::div_image(a,b,mult,add); });
  });
}

extern "C" ::MvStatus mv_invert_image(::MvObject Image,::MvObject* ImageInvert) {
  return mv::c_api::translate([&] {
    constexpr auto op="invert_image";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageInvert,op,"ImageInvert");
    auto result=mv::c_api::make_iconic(mv::imageproc::invert_image(Image->value));
    *ImageInvert=result.release();
  });
}

extern "C" ::MvStatus mv_crop_domain(::MvObject Image,::MvObject* ImagePart) {
  return mv::c_api::translate([&] {
    constexpr auto op="crop_domain";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImagePart,op,"ImagePart");
    auto result=mv::c_api::make_iconic(mv::imageproc::crop_domain(Image->value));
    *ImagePart=result.release();
  });
}

extern "C" ::MvStatus mv_paint_region(::MvObject Region,::MvObject Image,::MvObject* ImageResult,::MvTuple Grayval,::MvTuple Type) {
  return mv::c_api::translate([&]{
    constexpr auto op="paint_region";region_operand(Region,op,"Region");image_operand(Image,op,"Image");mv::c_api::require_output(ImageResult,op,"ImageResult");
    if(Region->value.empty()||Image->value.empty()){empty_image(ImageResult);return;}
    require_at_most_one(Image->value,op,"Image");
    const mv::c_api::TupleInput ti(Type,op,"Type"),gi(Grayval,op,"Grayval");
    if(ti.size()==0)ti.fail(mv::core::ErrorCode::InvalidCardinality,-1,"a paint type is required");const auto name=ti.string(0);
    mv::imageproc::PaintMode mode;
    if(name=="fill")mode=mv::imageproc::PaintMode::Fill;else if(name=="margin")mode=mv::imageproc::PaintMode::Margin;else ti.fail(mv::core::ErrorCode::InvalidValue,0,"unsupported paint type");
    const auto channels=Image->value.at(0).as_image().channel_count();
    if(gi.size()==0||gi.size()%channels)gi.fail(mv::core::ErrorCode::InvalidCardinality,-1,"gray values must contain whole channel groups");
    const auto groups=gi.size()/channels;
    if(groups!=1&&groups<Region->value.size())gi.fail(mv::core::ErrorCode::InvalidCardinality,-1,"provide one channel group or at least one group per region");
    const auto used=groups>=Region->value.size()?Region->value.size()*channels:channels;
    mv::core::Tuple gray;for(std::size_t i=0;i<used;++i){const auto value=gi.number(i);if(!std::isfinite(value))gi.fail(mv::core::ErrorCode::InvalidValue,i,"finite gray value required");gray.append(value);}
    auto result=mv::c_api::make_iconic(mv::imageproc::paint_region(mv::core::Regions{Region->value},Image->value,gray,mode));*ImageResult=result.release();
  });
}

namespace {
bool image_boolean(::MvTuple value,const char* op,const char* name) {
  const mv::c_api::TupleInput input(value,op,name);const auto text=mv::c_api::image_string(input);
  if(text=="true")return true;if(text=="false")return false;
  input.fail(mv::core::ErrorCode::InvalidValue,0,"true or false required");
}
std::size_t image_dimension(::MvTuple value,const char* op,const char* name) {
  const mv::c_api::TupleInput input(value,op,name);const auto number=mv::c_api::image_scalar_integer(input);
  if(number<=0||number>INT32_MAX)input.fail(mv::core::ErrorCode::InvalidValue,0,"positive dimension within int32 range required");
  return static_cast<std::size_t>(number);
}
}
extern "C" ::MvStatus mv_affine_trans_image(::MvObject Image,::MvObject* ImageAffineTrans,::MvTuple HomMat2D,::MvTuple Interpolation,::MvTuple AdaptImageSize) {
  return mv::c_api::translate([&]{constexpr auto op="affine_trans_image";image_operand(Image,op,"Image");mv::c_api::require_output(ImageAffineTrans,op,"ImageAffineTrans");
    const mv::c_api::TupleInput input_matrix(HomMat2D,op,"HomMat2D");
    if(input_matrix.size()!=6&&input_matrix.size()!=9)input_matrix.fail(mv::core::ErrorCode::InvalidCardinality,-1,"six or nine coefficients required");
    const auto mode=image_interpolation(mv::c_api::TupleInput(Interpolation,op,"Interpolation"));
    const auto adapt_text=mv::c_api::image_string(mv::c_api::TupleInput(AdaptImageSize,op,"AdaptImageSize"));const bool adapt=adapt_text=="true";
    if(Image->value.empty()){empty_image(ImageAffineTrans);return;}
    if(input_matrix.size()==9&&(input_matrix.number(6)!=0||input_matrix.number(7)!=0||input_matrix.number(8)!=1))input_matrix.fail(mv::core::ErrorCode::InvalidValue,-1,"affine matrix requires last row (0,0,1)");
    mv::core::Tuple matrix;for(std::size_t i=0;i<6;++i){const auto v=input_matrix.number(i);if(!std::isfinite(v))input_matrix.fail(mv::core::ErrorCode::InvalidValue,i,"finite coefficient required");matrix.append(v);}
    std::vector<mv::core::Image> result;for(const auto& v:Image->value.values())result.push_back(mv::imageproc::transform_image_halcon(v.as_image(),matrix,mode,false,adapt,false));
    auto owned=mv::c_api::make_iconic(mv::core::make_image_tuple(std::move(result)));*ImageAffineTrans=owned.release();
  });
}

extern "C" ::MvStatus mv_projective_trans_image(::MvObject Image,::MvObject* TransImage,::MvTuple HomMat2D,::MvTuple Interpolation,::MvTuple AdaptImageSize,::MvTuple TransformDomain) {
  return mv::c_api::translate([&]{constexpr auto op="projective_trans_image";image_operand(Image,op,"Image");mv::c_api::require_output(TransImage,op,"TransImage");
    if(Image->value.empty()){empty_image(TransImage);return;}
    const auto matrix=mv::c_api::TupleInput(HomMat2D,op,"HomMat2D").materialize();const auto mode=image_interpolation(mv::c_api::TupleInput(Interpolation,op,"Interpolation"));
    if(mode!=mv::imageproc::Interpolation::NearestNeighbor&&mode!=mv::imageproc::Interpolation::Bilinear)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"Interpolation",0,"nearest_neighbor or bilinear required");
    const auto adapt=image_boolean(AdaptImageSize,op,"AdaptImageSize"),domain=image_boolean(TransformDomain,op,"TransformDomain");
    std::vector<mv::core::Image> result;for(const auto& v:Image->value.values())result.push_back(mv::imageproc::transform_image_halcon(v.as_image(),matrix,mode,true,adapt,domain));
    auto owned=mv::c_api::make_iconic(mv::core::make_image_tuple(std::move(result)));*TransImage=owned.release();
  });
}

extern "C" ::MvStatus mv_rotate_image(::MvObject Image,::MvObject* ImageRotate,::MvTuple Phi,::MvTuple Interpolation) {
  return mv::c_api::translate([&] {
    constexpr auto op="rotate_image";image_operand(Image,op,"Image");mv::c_api::require_output(ImageRotate,op,"ImageRotate");
    const auto phi=mv::c_api::TupleInput(Phi,op,"Phi").scalar_number();
    const auto mode=image_interpolation(mv::c_api::TupleInput(Interpolation,op,"Interpolation"));
    constexpr double radians_per_degree=3.14159265358979323846/180.;
    const auto normalized=std::remainder(phi,360.);const bool right_angle=std::abs(normalized-90.*std::round(normalized/90.))<1e-10;
    if(normalized!=0)for(const auto& object:Image->value.values())for(std::size_t ch=0;ch<object.as_image().channel_count();++ch)
      if(object.as_image().pixel_type(ch)==mv::core::PixelType::Int32)throw Exception(Status::TypeMismatch,mv::core::ErrorCode::InvalidType,op,"Image",-1,"int4 is supported only by the zero-degree identity");
    if(!right_angle&&mode==mv::imageproc::Interpolation::Weighted)throw Exception(Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"Interpolation",0,"general-angle weighted interpolation is not implemented");
    auto result=mv::c_api::make_iconic(mv::imageproc::rotate_image(Image->value,-normalized*radians_per_degree,right_angle?mv::imageproc::Interpolation::NearestNeighbor:mode));
    *ImageRotate=result.release();
  });
}
extern "C" ::MvStatus mv_map_image(::MvObject Image,::MvObject Map,::MvObject* ImageMapped) {
  return mv::c_api::translate([&]{constexpr auto op="map_image";image_operand(Image,op,"Image");image_operand(Map,op,"Map");mv::c_api::require_output(ImageMapped,op,"ImageMapped");
    if(Image->value.empty()||Map->value.empty()){empty_image(ImageMapped);return;}
    require_at_most_one(Map->value,op,"Map");
    std::vector<mv::core::Image> result;for(const auto& v:Image->value.values())result.push_back(mv::imageproc::map_image_linear(v.as_image(),Map->value.at(0).as_image()));
    auto owned=mv::c_api::make_iconic(mv::core::make_image_tuple(std::move(result)));*ImageMapped=owned.release();
  });
}

extern "C" ::MvStatus mv_polar_trans_image_ext(::MvObject Image,::MvObject* PolarTransImage,::MvTuple Row,::MvTuple Column,::MvTuple AngleStart,::MvTuple AngleEnd,::MvTuple RadiusStart,::MvTuple RadiusEnd,::MvTuple Width,::MvTuple Height,::MvTuple Interpolation) {
  return mv::c_api::translate([&]{constexpr auto op="polar_trans_image_ext";image_operand(Image,op,"Image");mv::c_api::require_output(PolarTransImage,op,"PolarTransImage");
    if(Image->value.empty()){empty_image(PolarTransImage);return;}
    const auto row=mv::c_api::TupleInput(Row,op,"Row").scalar_number(),col=mv::c_api::TupleInput(Column,op,"Column").scalar_number();
    const auto as=mv::c_api::TupleInput(AngleStart,op,"AngleStart").scalar_number(),ae=mv::c_api::TupleInput(AngleEnd,op,"AngleEnd").scalar_number();
    const auto rs=mv::c_api::TupleInput(RadiusStart,op,"RadiusStart").scalar_number(),re=mv::c_api::TupleInput(RadiusEnd,op,"RadiusEnd").scalar_number();
    const auto width=image_dimension(Width,op,"Width"),height=image_dimension(Height,op,"Height");const auto mode=image_interpolation(mv::c_api::TupleInput(Interpolation,op,"Interpolation"));
    if(mode!=mv::imageproc::Interpolation::NearestNeighbor&&mode!=mv::imageproc::Interpolation::Bilinear)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"Interpolation",0,"nearest_neighbor or bilinear required");
    std::vector<mv::core::Image> result;for(const auto& v:Image->value.values())result.push_back(mv::imageproc::polar_trans_image_halcon(v.as_image(),row,col,as,ae,rs,re,width,height,mode));
    auto owned=mv::c_api::make_iconic(mv::core::make_image_tuple(std::move(result)));*PolarTransImage=owned.release();
  });
}

extern "C" ::MvStatus mv_gray_erosion_rect(::MvObject Image, ::MvObject* ImageMin, ::MvTuple MaskHeight, ::MvTuple MaskWidth) {
  return mv::c_api::translate([&] {
    constexpr auto op="gray_erosion_rect";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageMin,op,"ImageMin");
    const auto width=mv::c_api::TupleInput(MaskWidth,op,"MaskWidth").scalar_integer();
    if(width<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskWidth",0,"positive integer required");
    const auto height=mv::c_api::TupleInput(MaskHeight,op,"MaskHeight").scalar_integer();
    if(height<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskHeight",0,"positive integer required");
    auto result=mv::c_api::make_iconic(mv::imageproc::gray_erosion_rect(Image->value,static_cast<std::size_t>(width),static_cast<std::size_t>(height)));
    *ImageMin=result.release();
  });
}

extern "C" ::MvStatus mv_gray_dilation_rect(::MvObject Image, ::MvObject* ImageMax, ::MvTuple MaskHeight, ::MvTuple MaskWidth) {
  return mv::c_api::translate([&] {
    constexpr auto op="gray_dilation_rect";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageMax,op,"ImageMax");
    const auto width=mv::c_api::TupleInput(MaskWidth,op,"MaskWidth").scalar_integer();
    if(width<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskWidth",0,"positive integer required");
    const auto height=mv::c_api::TupleInput(MaskHeight,op,"MaskHeight").scalar_integer();
    if(height<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskHeight",0,"positive integer required");
    auto result=mv::c_api::make_iconic(mv::imageproc::gray_dilation_rect(Image->value,static_cast<std::size_t>(width),static_cast<std::size_t>(height)));
    *ImageMax=result.release();
  });
}

extern "C" ::MvStatus mv_gray_opening_rect(::MvObject Image, ::MvObject* ImageOpening, ::MvTuple MaskHeight, ::MvTuple MaskWidth) {
  return mv::c_api::translate([&] {
    constexpr auto op="gray_opening_rect";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageOpening,op,"ImageOpening");
    const auto width=mv::c_api::TupleInput(MaskWidth,op,"MaskWidth").scalar_integer();
    if(width<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskWidth",0,"positive integer required");
    const auto height=mv::c_api::TupleInput(MaskHeight,op,"MaskHeight").scalar_integer();
    if(height<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskHeight",0,"positive integer required");
    auto result=mv::c_api::make_iconic(mv::imageproc::gray_opening_rect(Image->value,static_cast<std::size_t>(width),static_cast<std::size_t>(height)));
    *ImageOpening=result.release();
  });
}

extern "C" ::MvStatus mv_gray_closing_rect(::MvObject Image, ::MvObject* ImageClosing, ::MvTuple MaskHeight, ::MvTuple MaskWidth) {
  return mv::c_api::translate([&] {
    constexpr auto op="gray_closing_rect";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageClosing,op,"ImageClosing");
    const auto width=mv::c_api::TupleInput(MaskWidth,op,"MaskWidth").scalar_integer();
    if(width<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskWidth",0,"positive integer required");
    const auto height=mv::c_api::TupleInput(MaskHeight,op,"MaskHeight").scalar_integer();
    if(height<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskHeight",0,"positive integer required");
    auto result=mv::c_api::make_iconic(mv::imageproc::gray_closing_rect(Image->value,static_cast<std::size_t>(width),static_cast<std::size_t>(height)));
    *ImageClosing=result.release();
  });
}

extern "C" ::MvStatus mv_laplace_of_gauss(::MvObject Image, ::MvObject* ImageLaplace, ::MvTuple Sigma) {
  return mv::c_api::translate([&] {
    constexpr auto op="laplace_of_gauss";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageLaplace,op,"ImageLaplace");
    const auto sigma=mv::c_api::TupleInput(Sigma,op,"Sigma").scalar_number();
    auto result=mv::c_api::make_iconic(mv::imageproc::laplace_of_gauss_standard(Image->value,sigma));
    *ImageLaplace=result.release();
  });
}

extern "C" ::MvStatus mv_emphasize(::MvObject Image, ::MvObject* ImageEmphasize, ::MvTuple MaskWidth, ::MvTuple MaskHeight, ::MvTuple Factor) {
  return mv::c_api::translate([&] {
    constexpr auto op="emphasize";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageEmphasize,op,"ImageEmphasize");
    Image->value.require_kind(mv::core::IconicKind::Image,op);
    for(const auto& value:Image->value.values())for(std::size_t c=0;c<value.as_image().channel_count();++c)
      if(value.as_image().pixel_type(c)==mv::core::PixelType::Int32||value.as_image().pixel_type(c)==mv::core::PixelType::Float32)
        throw Exception(Status::Unsupported,mv::core::ErrorCode::InvalidType,op,"Image",-1,"emphasize supports byte, uint2 and int2 images");
    const auto width=mv::c_api::TupleInput(MaskWidth,op,"MaskWidth").scalar_integer();
    if(width<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskWidth",0,"positive integer required");
    const auto height=mv::c_api::TupleInput(MaskHeight,op,"MaskHeight").scalar_integer();
    if(height<=0)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"MaskHeight",0,"positive integer required");
    const auto factor=mv::c_api::TupleInput(Factor,op,"Factor").scalar_number();
    auto result=mv::c_api::make_iconic(mv::imageproc::emphasize(Image->value,static_cast<std::size_t>(width),static_cast<std::size_t>(height),factor));
    *ImageEmphasize=result.release();
  });
}

extern "C" ::MvStatus mv_auto_threshold(::MvObject Image,::MvObject* Regions,::MvTuple Sigma) {
  return mv::c_api::translate([&] {
    constexpr auto op="auto_threshold";image_operand(Image,op,"Image");mv::c_api::require_output(Regions,op,"Regions");
    if(Image->value.empty()){empty_image(Regions);return;}
    const auto sigma=mv::c_api::TupleInput(Sigma,op,"Sigma").scalar_number();
    std::vector<mv::core::Region> regions;
    for(const auto& object:Image->value.values()) {
      const auto image=mv::imageproc::access_channel(object.as_image(),1);const auto type=image.pixel_type();
      if(type!=mv::core::PixelType::UInt8&&type!=mv::core::PixelType::UInt16&&type!=mv::core::PixelType::Float32)throw Exception(Status::TypeMismatch,mv::core::ErrorCode::InvalidType,op,"Image",-1,"expected byte, uint2 or real image");
      if(sigma<0||sigma>32)throw Exception(Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"Sigma",0,"histogram sigma outside [0,32] is not implemented");
      if(image.domain().empty()){regions.emplace_back();continue;}
      if(type==mv::core::PixelType::Float32)throw Exception(Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"Image",-1,"real histogram segmentation is not implemented");
      const auto result=mv::imageproc::auto_threshold(image,sigma);for(std::size_t i=0;i<result.size();++i)regions.push_back(result[i]);
    }
    *Regions=mv::c_api::make_regions(mv::core::Regions{std::move(regions)});
  });
}
extern "C" ::MvStatus mv_equ_histo_image(::MvObject Image, ::MvObject* ImageEquHisto) {
  return mv::c_api::translate([&] {
    constexpr auto op="equ_histo_image";
    mv::c_api::require_iconic(Image,op,"Image");
    mv::c_api::require_output(ImageEquHisto,op,"ImageEquHisto");
    auto result=mv::c_api::make_iconic(mv::imageproc::equ_histo_image(Image->value));
    *ImageEquHisto=result.release();
  });
}

namespace {
void color_outputs(std::array<::MvObject*,3> outputs,const char* op) {
  for(std::size_t i=0;i<outputs.size();++i) {
    mv::c_api::require_output(outputs[i],op,"ImageResult");
    const auto a=reinterpret_cast<std::uintptr_t>(outputs[i]);
    for(std::size_t j=0;j<i;++j) {
      const auto b=reinterpret_cast<std::uintptr_t>(outputs[j]);
      if((a<b?b-a:a-b)<sizeof(::MvObject))throw Exception(Status::InvalidArgument,
          mv::core::ErrorCode::InvalidValue,op,"ImageResult",i,"output ranges must be distinct");
    }
  }
}
void color_transform(::MvObject first,::MvObject second,::MvObject third,
    std::array<::MvObject*,3> outputs,::MvTuple space,bool inverse) {
  const auto* op=inverse?"trans_to_rgb":"trans_from_rgb";
  image_operand(first,op,"Image1");image_operand(second,op,"Image2");image_operand(third,op,"Image3");
  color_outputs(outputs,op);
  std::array<std::vector<mv::core::Image>,3> planes;
  if(first->value.size()!=second->value.size()||first->value.size()!=third->value.size())
    throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidCardinality,op,"Image2/Image3",-1,"identical object counts required");
  const auto name=mv::c_api::image_string(mv::c_api::TupleInput(space,op,"ColorSpace"));
  if(!first->value.empty()&&!second->value.empty()&&!third->value.empty()) {
    if(name!="hsv")throw Exception(Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"ColorSpace",0,"only hsv is implemented");
    const auto count=first->value.size();
    for(std::size_t i=0;i<count;++i) {
      const std::array<mv::core::Image,3> source{first->value.at(i).as_image(),second->value.at(i).as_image(),third->value.at(i).as_image()};
      const auto type=source[0].pixel_type();
      if(type!=mv::core::PixelType::UInt8&&type!=mv::core::PixelType::UInt16&&type!=mv::core::PixelType::Float32)
        throw Exception(Status::Unsupported,mv::core::ErrorCode::InvalidType,op,"Image",-1,"byte, uint2 and real HSV conversions are implemented");
      for(const auto& image:source)if(image.channel_count()!=1||image.pixel_type()!=type)
        throw Exception(Status::TypeMismatch,mv::core::ErrorCode::InvalidType,op,"Image",-1,"single-channel inputs of identical type required");
      const std::array<double,3> factors{type==mv::core::PixelType::UInt8?255.:type==mv::core::PixelType::UInt16?21600.:6.2831853071795864769,type==mv::core::PixelType::UInt8?255.:type==mv::core::PixelType::UInt16?10000.:1.,type==mv::core::PixelType::UInt8?255.:type==mv::core::PixelType::UInt16?65535.:1.};
      if(const auto fused=mv::imageproc::try_transform_byte_planes(source,inverse)) {
        for(std::size_t ch=0;ch<3;++ch)planes[ch].push_back(mv::imageproc::access_channel(*fused,ch+1));
      } else if(inverse) {
        std::array<mv::core::Image,3> normalized;
        for(std::size_t ch=0;ch<3;++ch)normalized[ch]=mv::imageproc::scale_image(mv::imageproc::convert_image_type(source[ch],mv::core::PixelType::Float32),1./factors[ch],0);
        const auto rgb=mv::imageproc::trans_to_rgb(mv::imageproc::compose3(normalized[0],normalized[1],normalized[2]),type);
        for(std::size_t ch=0;ch<3;++ch)planes[ch].push_back(mv::imageproc::access_channel(rgb,ch+1));
      } else {
        const auto hsv=mv::imageproc::trans_from_rgb(mv::imageproc::compose3(source[0],source[1],source[2]));
        for(std::size_t ch=0;ch<3;++ch)planes[ch].push_back(mv::imageproc::convert_image_type(mv::imageproc::scale_image(mv::imageproc::access_channel(hsv,ch+1),factors[ch],0),type));
      }
    }
  }
  std::array<std::unique_ptr<MvObjectImpl>,3> staged;
  for(std::size_t ch=0;ch<3;++ch)staged[ch]=mv::c_api::make_iconic(mv::core::make_image_tuple(std::move(planes[ch])));
  for(std::size_t ch=0;ch<3;++ch)*outputs[ch]=staged[ch].release();
}
mv::imageproc::ThresholdPolarity local_polarity(const mv::c_api::TupleInput& input) {
  const auto name=mv::c_api::image_string(input);
  if(name=="light")return mv::imageproc::ThresholdPolarity::Light;
  if(name=="dark")return mv::imageproc::ThresholdPolarity::Dark;
  input.fail(mv::core::ErrorCode::InvalidValue,0,"light or dark required");
}
}
extern "C" ::MvStatus mv_trans_from_rgb(::MvObject ImageRed,::MvObject ImageGreen,::MvObject ImageBlue,::MvObject* ImageResult1,::MvObject* ImageResult2,::MvObject* ImageResult3,::MvTuple ColorSpace) {
  return mv::c_api::translate([&]{color_transform(ImageRed,ImageGreen,ImageBlue,{ImageResult1,ImageResult2,ImageResult3},ColorSpace,false);});
}
extern "C" ::MvStatus mv_trans_to_rgb(::MvObject ImageInput1,::MvObject ImageInput2,::MvObject ImageInput3,::MvObject* ImageRed,::MvObject* ImageGreen,::MvObject* ImageBlue,::MvTuple ColorSpace) {
  return mv::c_api::translate([&]{color_transform(ImageInput1,ImageInput2,ImageInput3,{ImageRed,ImageGreen,ImageBlue},ColorSpace,true);});
}
extern "C" ::MvStatus mv_derivate_gauss(::MvObject Image,::MvObject* DerivGauss,::MvTuple Sigma,::MvTuple Component) {
  return mv::c_api::translate([&] {
    constexpr auto op="derivate_gauss";image_operand(Image,op,"Image");mv::c_api::require_output(DerivGauss,op,"DerivGauss");
    const auto name=mv::c_api::image_string(mv::c_api::TupleInput(Component,op,"Component"));
    const mv::c_api::TupleInput sigmas(Sigma,op,"Sigma");
    if(sigmas.size()!=1&&sigmas.size()!=2)sigmas.fail(mv::core::ErrorCode::InvalidCardinality,-1,"one or two sigma values required");
    const auto sx=sigmas.number(0),sy=sigmas.number(sigmas.size()==1?0:1);
    if(!std::isfinite(sx)||!std::isfinite(sy)||sx<.1||sy<.1||sx>32||sy>32)
      sigmas.fail(mv::core::ErrorCode::InvalidValue,-1,"implemented sigma range is [0.1,32]");
    if(Image->value.empty()){empty_image(DerivGauss);return;}
    std::size_t ox=0,oy=0;
    if(name=="x")ox=1;else if(name=="y")oy=1;else if(name=="xx")ox=2;else if(name=="yy")oy=2;else if(name=="xy"){ox=1;oy=1;}
    else if(name!="none")throw Exception(Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"Component",0,"only none, x, y, xx, yy and xy are implemented");
    std::vector<mv::core::Image> values;
    for(const auto& object:Image->value.values())values.push_back(mv::imageproc::derivate_gauss(object.as_image(),sx,sy,ox,oy));
    *DerivGauss=mv::c_api::make_iconic(mv::core::make_image_tuple(std::move(values))).release();
  });
}

extern "C" ::MvStatus mv_local_threshold(::MvObject Image,::MvObject* Region,::MvTuple Method,::MvTuple LightDark,::MvTuple GenParamName,::MvTuple GenParamValue) {
  return mv::c_api::translate([&] {
    constexpr auto op="local_threshold";image_operand(Image,op,"Image");mv::c_api::require_output(Region,op,"Region");
    if(Image->value.empty()){empty_image(Region);return;}
    const auto method=mv::c_api::image_string(mv::c_api::TupleInput(Method,op,"Method"));
    if(method!="adapted_std_deviation")throw Exception(Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"Method",0,"only adapted_std_deviation is implemented");
    const auto polarity=local_polarity(mv::c_api::TupleInput(LightDark,op,"LightDark"));
    const mv::c_api::TupleInput names(GenParamName,op,"GenParamName"),params(GenParamValue,op,"GenParamValue");
    if(names.size()!=params.size())params.fail(mv::core::ErrorCode::InvalidCardinality,-1,"generic name/value counts must match");
    std::int64_t mask=15;double k=.2,range=0;bool custom_range=false;
    for(std::size_t i=0;i<names.size();++i) {
      const auto name=names.string(i);
      if(name=="mask_size")mask=mv::c_api::image_integer(params,i);
      else if(name=="scale")k=params.number(i);
      else if(name=="range"){range=params.number(i);custom_range=true;}
      else names.fail(mv::core::ErrorCode::InvalidValue,i,"unknown generic parameter");
    }
    if(mask<1||mask>4095)throw Exception(Status::InvalidArgument,mv::core::ErrorCode::InvalidValue,op,"mask_size",-1,"mask_size in [1,4095] required");
    if(!(mask&1))++mask;
    std::vector<mv::core::Region> regions;
    for(const auto& object:Image->value.values()) {
      const auto image=mv::imageproc::access_channel(object.as_image(),1);const auto type=image.pixel_type();
      if(type!=mv::core::PixelType::UInt8&&type!=mv::core::PixelType::UInt16)throw Exception(Status::TypeMismatch,mv::core::ErrorCode::InvalidType,op,"Image",-1,"byte or uint2 first channel required");
      regions.push_back(mv::imageproc::local_threshold(image,mask,mask,k,custom_range?range:type==mv::core::PixelType::UInt8?128.:32767.5,polarity));
    }
    *Region=mv::c_api::make_regions(mv::core::Regions{std::move(regions)});
  });
}
