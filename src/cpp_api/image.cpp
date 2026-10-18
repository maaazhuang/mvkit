#include "api_bridge.hpp"
#include <mvkit/cpp/mv_image.hpp>
#include <mvkit/cpp/mv_geometry.hpp>
#include <mvkit/c/mv_image.h>
#include <mvkit/c/mv_image_processing.h>
#include <utility>
#include <cstring>
#include <vector>

namespace mv {
std::size_t MvImage::Channels() const {
  MvTuple channels;
  mv::CountChannels(*this, &channels);
  if (channels.Length() != 1) throw MvException(MvStatus::InvalidArgument, MvErrorCode::InvalidCardinality, "Channels", "Image", -1, "one image required");
  return static_cast<std::size_t>(channels.I());
}
std::size_t MvImage::Stride(std::size_t channel) const {
  const auto type = Type(channel);
  const std::size_t bytes = type == MvPixelType::UInt8 ? 1 : (type == MvPixelType::UInt16 || type == MvPixelType::Int16) ? 2 : 4;
  return Width() * bytes;
}

namespace {
[[noreturn]] void invalid_outputs(const char* op) {
  throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidValue,{op,std::strlen(op)},"Output",-1,"output pointers must be distinct");
}
[[noreturn]] void invalid_scalar_output(const char* op) {
  throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,{op,std::strlen(op)},"Region",-1,"scalar output requires exactly one region/image result");
}
template<class Call> void image_output(MvObject* result,const char* op,Call&& call) {
  cpp_api::require_output(result,op);
  ::MvObject handle=nullptr;cpp_api::check_status(call(&handle),op);
  detail::MvObjectAccess::Commit(result,detail::MvObjectAccess::Adopt(handle));
}
template<class Call> void pair_output(MvTuple* first,MvTuple* second,const char* op,Call&& call) {
  cpp_api::TupleOutputs<2> outputs(op,{{first,second}});
  cpp_api::check_status(call(outputs.Out(0),outputs.Out(1)),op);
  outputs.Commit();
}
}
void GetDomain(const MvObject& Image,MvObject* Domain) {
  image_output(Domain,"GetDomain",[&](::MvObject* h){return mv_get_domain(Image.NativeHandle(),h);});
}
void ReduceDomain(const MvObject& Image,const MvObject& Region,MvObject* ImageReduced) {
  image_output(ImageReduced,"ReduceDomain",[&](::MvObject* h){return mv_reduce_domain(Image.NativeHandle(),Region.NativeHandle(),h);});
}
void FullDomain(const MvObject& Image,MvObject* ImageFull) {
  image_output(ImageFull,"FullDomain",[&](::MvObject* h){return mv_full_domain(Image.NativeHandle(),h);});
}
void InvertImage(const MvObject& Image,MvObject* ImageInvert) {
  image_output(ImageInvert,"InvertImage",[&](::MvObject* h){return mv_invert_image(Image.NativeHandle(),h);});
}
void Rgb1ToGray(const MvObject& RGBImage,MvObject* GrayImage) {
  image_output(GrayImage,"Rgb1ToGray",[&](::MvObject* h){return mv_rgb1_to_gray(RGBImage.NativeHandle(),h);});
}
void CropDomain(const MvObject& Image,MvObject* ImagePart) {
  image_output(ImagePart,"CropDomain",[&](::MvObject* h){return mv_crop_domain(Image.NativeHandle(),h);});
}
void ConvertImageType(const MvObject& Image,MvObject* ImageConverted,const MvTuple& NewType) {
  image_output(ImageConverted,"ConvertImageType",[&](::MvObject* h){return mv_convert_image_type(Image.NativeHandle(),h,NewType.ControlInput());});
}
void AccessChannel(const MvObject& MultiChannelImage,MvObject* Image,const MvTuple& Channel) {
  image_output(Image,"AccessChannel",[&](::MvObject* h){return mv_access_channel(MultiChannelImage.NativeHandle(),h,Channel.ControlInput());});
}
void Compose3(const MvObject& Image1,const MvObject& Image2,const MvObject& Image3,MvObject* MultiChannelImage) {
  image_output(MultiChannelImage,"Compose3",[&](::MvObject* h){return mv_compose3(Image1.NativeHandle(),Image2.NativeHandle(),Image3.NativeHandle(),h);});
}
void Decompose3(const MvObject& MultiChannelImage,MvObject* Image1,MvObject* Image2,MvObject* Image3) {
  cpp_api::require_output(Image1,"Decompose3");cpp_api::require_output(Image2,"Decompose3");cpp_api::require_output(Image3,"Decompose3");
  if(Image1==Image2||Image1==Image3||Image2==Image3)invalid_outputs("Decompose3");
  ::MvObject a=nullptr,b=nullptr,c=nullptr;
  cpp_api::check_status(mv_decompose3(MultiChannelImage.NativeHandle(),&a,&b,&c),"Decompose3");
  auto ra=detail::MvObjectAccess::Adopt(a),rb=detail::MvObjectAccess::Adopt(b),rc=detail::MvObjectAccess::Adopt(c);
  detail::MvObjectAccess::CheckOutput(Image1,ra);detail::MvObjectAccess::CheckOutput(Image2,rb);detail::MvObjectAccess::CheckOutput(Image3,rc);
  detail::MvObjectAccess::Commit(Image1,std::move(ra));detail::MvObjectAccess::Commit(Image2,std::move(rb));detail::MvObjectAccess::Commit(Image3,std::move(rc));
}
void Intensity(const MvObject& Regions,const MvObject& Image,MvTuple* Mean,MvTuple* Deviation) {
  pair_output(Mean,Deviation,"Intensity",[&](::MvTuple* a,::MvTuple* b){return mv_intensity(Regions.NativeHandle(),Image.NativeHandle(),a,b);});
}
void GrayHisto(const MvObject& Region,const MvObject& Image,MvTuple* AbsoluteHisto,MvTuple* RelativeHisto) {
  pair_output(AbsoluteHisto,RelativeHisto,"GrayHisto",[&](::MvTuple* a,::MvTuple* b){return mv_gray_histo(Region.NativeHandle(),Image.NativeHandle(),a,b);});
}
MvRegion MvImage::GetDomain() const { MvRegion result;mv::GetDomain(*this,&result);return result; }
MvImage MvImage::FullDomain() const { MvImage result;mv::FullDomain(*this,&result);return result; }
MvImage MvImage::InvertImage() const { MvImage result;mv::InvertImage(*this,&result);return result; }
MvImage MvImage::Rgb1ToGray() const { MvImage result;mv::Rgb1ToGray(*this,&result);return result; }
MvImage MvImage::CropDomain() const { MvImage result;mv::CropDomain(*this,&result);return result; }
MvImage MvImage::ReduceDomain(const MvRegion& region) const { MvImage result;mv::ReduceDomain(*this,region,&result);return result; }
MvImage MvImage::ConvertImageType(const MvString& type) const { MvImage result;mv::ConvertImageType(*this,&result,MvTuple(type));return result; }
MvImage MvImage::ConvertImageType(const char* type) const { return ConvertImageType(MvString(type)); }
MvImage MvImage::AccessChannel(std::int64_t channel) const { MvImage result;mv::AccessChannel(*this,&result,MvTuple(channel));return result; }
MvImage MvImage::Compose3(const MvImage& second,const MvImage& third) const { MvImage result;mv::Compose3(*this,second,third,&result);return result; }
MvImage MvImage::Decompose3(MvImage* second,MvImage* third) const { MvImage result;mv::Decompose3(*this,&result,second,third);return result; }
MvTuple MvImage::Intensity(const MvRegion& region,MvTuple* deviation) const { MvTuple result;mv::Intensity(region,*this,&result,deviation);return result; }
double MvImage::Intensity(const MvRegion& region,double* deviation) const { cpp_api::require_output(deviation,"Intensity");MvTuple d;auto result=Intensity(region,&d);if(result.Length()!=1||d.Length()!=1)invalid_scalar_output("Intensity");const auto value=result.D();*deviation=d.D();return value; }
MvTuple MvImage::GrayHisto(const MvRegion& region,MvTuple* relative) const { MvTuple result;mv::GrayHisto(region,*this,&result,relative);return result; }
MvTuple MvRegion::Intensity(const MvImage& image,MvTuple* deviation) const { return image.Intensity(*this,deviation); }
double MvRegion::Intensity(const MvImage& image,double* deviation) const { return image.Intensity(*this,deviation); }
MvTuple MvRegion::GrayHisto(const MvImage& image,MvTuple* relative) const { return image.GrayHisto(*this,relative); }
void MeanImage(const MvObject& Image, MvObject* ImageMean, const MvTuple& MaskWidth, const MvTuple& MaskHeight) {
  image_output(ImageMean,"MeanImage",[&](::MvObject* h){return mv_mean_image(Image.NativeHandle(),h,MaskWidth.ControlInput(),MaskHeight.ControlInput());});
}
MvImage MvImage::MeanImage(std::int64_t MaskWidth, std::int64_t MaskHeight) const { MvImage result;mv::MeanImage(*this,&result,MvTuple(MaskWidth),MvTuple(MaskHeight));return result; }
void GaussFilter(const MvObject& Image, MvObject* ImageGauss, const MvTuple& Size) {
  image_output(ImageGauss,"GaussFilter",[&](::MvObject* h){return mv_gauss_filter(Image.NativeHandle(),h,Size.ControlInput());});
}
MvImage MvImage::GaussFilter(std::int64_t Size) const { MvImage result;mv::GaussFilter(*this,&result,MvTuple(Size));return result; }
void MedianImage(const MvObject& Image,MvObject* ImageMedian,const MvTuple& MaskType,const MvTuple& Radius,const MvTuple& Margin) {
  image_output(ImageMedian,"MedianImage",[&](::MvObject* h){return mv_median_image(Image.NativeHandle(),h,MaskType.ControlInput(),Radius.ControlInput(),Margin.ControlInput());});
}
MvImage MvImage::MedianImage(const MvString& MaskType,std::int64_t Radius,const MvTuple& Margin) const {
  MvImage result;mv::MedianImage(*this,&result,MvTuple(MaskType),MvTuple(Radius),Margin);return result;
}
MvImage MvImage::MedianImage(const MvString& MaskType,std::int64_t Radius,const MvString& Margin) const {
  return MedianImage(MaskType,Radius,MvTuple(Margin));
}
MvImage MvImage::MedianImage(const char* MaskType,std::int64_t Radius,const char* Margin) const {
  return MedianImage(MvString(MaskType),Radius,MvTuple(Margin));
}
void GrayErosionRect(const MvObject& Image, MvObject* ImageMin, const MvTuple& MaskHeight, const MvTuple& MaskWidth) {
  image_output(ImageMin,"GrayErosionRect",[&](::MvObject* h){return mv_gray_erosion_rect(Image.NativeHandle(),h,MaskHeight.ControlInput(),MaskWidth.ControlInput());});
}
MvImage MvImage::GrayErosionRect(std::int64_t MaskHeight, std::int64_t MaskWidth) const { MvImage result;mv::GrayErosionRect(*this,&result,MvTuple(MaskHeight),MvTuple(MaskWidth));return result; }
void GrayDilationRect(const MvObject& Image, MvObject* ImageMax, const MvTuple& MaskHeight, const MvTuple& MaskWidth) {
  image_output(ImageMax,"GrayDilationRect",[&](::MvObject* h){return mv_gray_dilation_rect(Image.NativeHandle(),h,MaskHeight.ControlInput(),MaskWidth.ControlInput());});
}
MvImage MvImage::GrayDilationRect(std::int64_t MaskHeight, std::int64_t MaskWidth) const { MvImage result;mv::GrayDilationRect(*this,&result,MvTuple(MaskHeight),MvTuple(MaskWidth));return result; }
void GrayOpeningRect(const MvObject& Image, MvObject* ImageOpening, const MvTuple& MaskHeight, const MvTuple& MaskWidth) {
  image_output(ImageOpening,"GrayOpeningRect",[&](::MvObject* h){return mv_gray_opening_rect(Image.NativeHandle(),h,MaskHeight.ControlInput(),MaskWidth.ControlInput());});
}
MvImage MvImage::GrayOpeningRect(std::int64_t MaskHeight, std::int64_t MaskWidth) const { MvImage result;mv::GrayOpeningRect(*this,&result,MvTuple(MaskHeight),MvTuple(MaskWidth));return result; }
void GrayClosingRect(const MvObject& Image, MvObject* ImageClosing, const MvTuple& MaskHeight, const MvTuple& MaskWidth) {
  image_output(ImageClosing,"GrayClosingRect",[&](::MvObject* h){return mv_gray_closing_rect(Image.NativeHandle(),h,MaskHeight.ControlInput(),MaskWidth.ControlInput());});
}
MvImage MvImage::GrayClosingRect(std::int64_t MaskHeight, std::int64_t MaskWidth) const { MvImage result;mv::GrayClosingRect(*this,&result,MvTuple(MaskHeight),MvTuple(MaskWidth));return result; }
void Emphasize(const MvObject& Image, MvObject* ImageEmphasize, const MvTuple& MaskWidth, const MvTuple& MaskHeight, const MvTuple& Factor) {
  image_output(ImageEmphasize,"Emphasize",[&](::MvObject* h){return mv_emphasize(Image.NativeHandle(),h,MaskWidth.ControlInput(),MaskHeight.ControlInput(),Factor.ControlInput());});
}
MvImage MvImage::Emphasize(std::int64_t MaskWidth, std::int64_t MaskHeight, double Factor) const { MvImage result;mv::Emphasize(*this,&result,MvTuple(MaskWidth),MvTuple(MaskHeight),MvTuple(Factor));return result; }
void LaplaceOfGauss(const MvObject& Image, MvObject* ImageLaplace, const MvTuple& Sigma) {
  image_output(ImageLaplace,"LaplaceOfGauss",[&](::MvObject* h){return mv_laplace_of_gauss(Image.NativeHandle(),h,Sigma.ControlInput());});
}
MvImage MvImage::LaplaceOfGauss(const MvTuple& Sigma) const { MvImage result;mv::LaplaceOfGauss(*this,&result,Sigma);return result; }
MvImage MvImage::LaplaceOfGauss(double Sigma) const { MvImage result;mv::LaplaceOfGauss(*this,&result,MvTuple(Sigma));return result; }
void EquHistoImage(const MvObject& Image, MvObject* ImageEquHisto) {
  image_output(ImageEquHisto,"EquHistoImage",[&](::MvObject* h){return mv_equ_histo_image(Image.NativeHandle(),h);});
}
MvImage MvImage::EquHistoImage() const { MvImage result;mv::EquHistoImage(*this,&result);return result; }
void ScaleImage(const MvObject& Image, MvObject* ImageScaled, const MvTuple& Mult, const MvTuple& Add) {
  image_output(ImageScaled,"ScaleImage",[&](::MvObject* h){return mv_scale_image(Image.NativeHandle(),h,Mult.ControlInput(),Add.ControlInput());});
}
MvImage MvImage::ScaleImage(const MvTuple& Mult, const MvTuple& Add) const { MvImage result;mv::ScaleImage(*this,&result,Mult,Add);return result; }
MvImage MvImage::ScaleImage(double Mult, double Add) const { MvImage result;mv::ScaleImage(*this,&result,MvTuple(Mult),MvTuple(Add));return result; }
void AbsDiffImage(const MvObject& Image1, const MvObject& Image2, MvObject* ImageAbsDiff, const MvTuple& Mult) {
  image_output(ImageAbsDiff,"AbsDiffImage",[&](::MvObject* h){return mv_abs_diff_image(Image1.NativeHandle(),Image2.NativeHandle(),h,Mult.ControlInput());});
}
MvImage MvImage::AbsDiffImage(const MvImage& Image2, const MvTuple& Mult) const { MvImage result;mv::AbsDiffImage(*this,Image2,&result,Mult);return result; }
MvImage MvImage::AbsDiffImage(const MvImage& Image2, double Mult) const { return AbsDiffImage(Image2,MvTuple(Mult)); }

void SubImage(const MvObject& ImageMinuend, const MvObject& ImageSubtrahend, MvObject* ImageSub, const MvTuple& Mult, const MvTuple& Add) {
  image_output(ImageSub,"SubImage",[&](::MvObject* h){return mv_sub_image(ImageMinuend.NativeHandle(),ImageSubtrahend.NativeHandle(),h,Mult.ControlInput(),Add.ControlInput());});
}
MvImage MvImage::SubImage(const MvImage& ImageSubtrahend, const MvTuple& Mult, const MvTuple& Add) const { MvImage result;mv::SubImage(*this,ImageSubtrahend,&result,Mult,Add);return result; }
MvImage MvImage::SubImage(const MvImage& ImageSubtrahend, double Mult, double Add) const { return SubImage(ImageSubtrahend,MvTuple(Mult),MvTuple(Add)); }

void MultImage(const MvObject& Image1, const MvObject& Image2, MvObject* ImageResult, const MvTuple& Mult, const MvTuple& Add) {
  image_output(ImageResult,"MultImage",[&](::MvObject* h){return mv_mult_image(Image1.NativeHandle(),Image2.NativeHandle(),h,Mult.ControlInput(),Add.ControlInput());});
}
MvImage MvImage::MultImage(const MvImage& Image2, const MvTuple& Mult, const MvTuple& Add) const { MvImage result;mv::MultImage(*this,Image2,&result,Mult,Add);return result; }
MvImage MvImage::MultImage(const MvImage& Image2, double Mult, double Add) const { return MultImage(Image2,MvTuple(Mult),MvTuple(Add)); }

void DivImage(const MvObject& Image1, const MvObject& Image2, MvObject* ImageResult, const MvTuple& Mult, const MvTuple& Add) {
  image_output(ImageResult,"DivImage",[&](::MvObject* h){return mv_div_image(Image1.NativeHandle(),Image2.NativeHandle(),h,Mult.ControlInput(),Add.ControlInput());});
}
MvImage MvImage::DivImage(const MvImage& Image2, const MvTuple& Mult, const MvTuple& Add) const { MvImage result;mv::DivImage(*this,Image2,&result,Mult,Add);return result; }
MvImage MvImage::DivImage(const MvImage& Image2, double Mult, double Add) const { return DivImage(Image2,MvTuple(Mult),MvTuple(Add)); }

namespace {
void min_max_scalar_outputs(double* a,double* b,double* c) {
  const double* outputs[]={a,b,c};
  for(std::size_t i=0;i<3;++i){cpp_api::require_output(outputs[i],"MinMaxGray");
    for(std::size_t j=0;j<i;++j){const auto x=reinterpret_cast<std::uintptr_t>(outputs[i]),y=reinterpret_cast<std::uintptr_t>(outputs[j]);
      if((x<y?y-x:x-y)<sizeof(double))invalid_outputs("MinMaxGray");}}
}
}
void MinMaxGray(const MvObject& Regions, const MvObject& Image, const MvTuple& Percent, MvTuple* Min, MvTuple* Max, MvTuple* Range) { cpp_api::TupleOutputs<3> outputs("MinMaxGray",{{Min,Max,Range}});cpp_api::check_status(mv_min_max_gray(Regions.NativeHandle(),Image.NativeHandle(),Percent.ControlInput(),outputs.Out(0),outputs.Out(1),outputs.Out(2)),"MinMaxGray");outputs.Commit(); }
void MvImage::MinMaxGray(const MvRegion& Regions, const MvTuple& Percent, MvTuple* Min, MvTuple* Max, MvTuple* Range) const { mv::MinMaxGray(Regions,*this,Percent,Min,Max,Range); }
void MvImage::MinMaxGray(const MvRegion& Regions, double Percent, double* Min, double* Max, double* Range) const { min_max_scalar_outputs(Min,Max,Range);MvTuple low,high,range;mv::MinMaxGray(Regions,*this,MvTuple(Percent),&low,&high,&range);if(low.Length()==0||high.Length()==0||range.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"MinMaxGray","Regions",-1,"scalar statistics require a nonempty result");const double a=low.D(),b=high.D(),c=range.D();*Min=a;*Max=b;*Range=c; }
void MvRegion::MinMaxGray(const MvImage& Image, const MvTuple& Percent, MvTuple* Min, MvTuple* Max, MvTuple* Range) const { mv::MinMaxGray(*this,Image,Percent,Min,Max,Range); }
void MvRegion::MinMaxGray(const MvImage& Image, double Percent, double* Min, double* Max, double* Range) const { min_max_scalar_outputs(Min,Max,Range);MvTuple low,high,range;mv::MinMaxGray(*this,Image,MvTuple(Percent),&low,&high,&range);if(low.Length()==0||high.Length()==0||range.Length()==0)throw MvException(MvStatus::InvalidArgument,MvErrorCode::InvalidCardinality,"MinMaxGray","Regions",-1,"scalar statistics require a nonempty result");const double a=low.D(),b=high.D(),c=range.D();*Min=a;*Max=b;*Range=c; }
void CropRectangle1(const MvObject& Image, MvObject* ImagePart, const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2) { image_output(ImagePart,"CropRectangle1",[&](::MvObject* h){return mv_crop_rectangle1(Image.NativeHandle(),h,Row1.ControlInput(),Column1.ControlInput(),Row2.ControlInput(),Column2.ControlInput());}); }
MvImage MvImage::CropRectangle1(const MvTuple& Row1, const MvTuple& Column1, const MvTuple& Row2, const MvTuple& Column2) const { MvImage result;mv::CropRectangle1(*this,&result,Row1,Column1,Row2,Column2);return result; }
MvImage MvImage::CropRectangle1(std::int64_t Row1, std::int64_t Column1, std::int64_t Row2, std::int64_t Column2) const { MvImage result;mv::CropRectangle1(*this,&result,MvTuple(Row1),MvTuple(Column1),MvTuple(Row2),MvTuple(Column2));return result; }
void MirrorImage(const MvObject& Image, MvObject* ImageMirror, const MvTuple& Mode) { image_output(ImageMirror,"MirrorImage",[&](::MvObject* h){return mv_mirror_image(Image.NativeHandle(),h,Mode.ControlInput());}); }
MvImage MvImage::MirrorImage(const MvString& Mode) const { MvImage result;mv::MirrorImage(*this,&result,MvTuple(MvString(Mode).Text()));return result; }
MvImage MvImage::MirrorImage(const char* Mode) const { MvImage result;mv::MirrorImage(*this,&result,MvTuple(MvString(Mode).Text()));return result; }
MvImage MvImage::MirrorImage(const wchar_t* Mode) const { MvImage result;mv::MirrorImage(*this,&result,MvTuple(MvString(Mode).Text()));return result; }
void ZoomImageSize(const MvObject& Image, MvObject* ImageZoom, const MvTuple& Width, const MvTuple& Height, const MvTuple& Interpolation) { image_output(ImageZoom,"ZoomImageSize",[&](::MvObject* h){return mv_zoom_image_size(Image.NativeHandle(),h,Width.ControlInput(),Height.ControlInput(),Interpolation.ControlInput());}); }
MvImage MvImage::ZoomImageSize(std::int64_t Width, std::int64_t Height, const MvString& Interpolation) const { MvImage result;mv::ZoomImageSize(*this,&result,MvTuple(Width),MvTuple(Height),MvTuple(MvString(Interpolation).Text()));return result; }
MvImage MvImage::ZoomImageSize(std::int64_t Width, std::int64_t Height, const char* Interpolation) const { MvImage result;mv::ZoomImageSize(*this,&result,MvTuple(Width),MvTuple(Height),MvTuple(MvString(Interpolation).Text()));return result; }
MvImage MvImage::ZoomImageSize(std::int64_t Width, std::int64_t Height, const wchar_t* Interpolation) const { MvImage result;mv::ZoomImageSize(*this,&result,MvTuple(Width),MvTuple(Height),MvTuple(MvString(Interpolation).Text()));return result; }
void SobelAmp(const MvObject& Image, MvObject* EdgeAmplitude, const MvTuple& FilterType, const MvTuple& Size) { image_output(EdgeAmplitude,"SobelAmp",[&](::MvObject* h){return mv_sobel_amp(Image.NativeHandle(),h,FilterType.ControlInput(),Size.ControlInput());}); }
MvImage MvImage::SobelAmp(const MvString& FilterType, const MvTuple& Size) const { MvImage result;mv::SobelAmp(*this,&result,MvTuple(MvString(FilterType).Text()),Size);return result; }
MvImage MvImage::SobelAmp(const MvString& FilterType, std::int64_t Size) const { MvImage result;mv::SobelAmp(*this,&result,MvTuple(MvString(FilterType).Text()),MvTuple(Size));return result; }
MvImage MvImage::SobelAmp(const char* FilterType, std::int64_t Size) const { MvImage result;mv::SobelAmp(*this,&result,MvTuple(MvString(FilterType).Text()),MvTuple(Size));return result; }
MvImage MvImage::SobelAmp(const wchar_t* FilterType, std::int64_t Size) const { MvImage result;mv::SobelAmp(*this,&result,MvTuple(MvString(FilterType).Text()),MvTuple(Size));return result; }
void DynThreshold(const MvObject& OrigImage, const MvObject& ThresholdImage, MvObject* RegionDynThresh, const MvTuple& Offset, const MvTuple& LightDark) { image_output(RegionDynThresh,"DynThreshold",[&](::MvObject* h){return mv_dyn_threshold(OrigImage.NativeHandle(),ThresholdImage.NativeHandle(),h,Offset.ControlInput(),LightDark.ControlInput());}); }
MvRegion MvImage::DynThreshold(const MvImage& ThresholdImage, const MvTuple& Offset, const MvString& LightDark) const { MvRegion result;mv::DynThreshold(*this,ThresholdImage,&result,Offset,MvTuple(MvString(LightDark).Text()));return result; }
MvRegion MvImage::DynThreshold(const MvImage& ThresholdImage, double Offset, const MvString& LightDark) const { MvRegion result;mv::DynThreshold(*this,ThresholdImage,&result,MvTuple(Offset),MvTuple(MvString(LightDark).Text()));return result; }
MvRegion MvImage::DynThreshold(const MvImage& ThresholdImage, double Offset, const char* LightDark) const { MvRegion result;mv::DynThreshold(*this,ThresholdImage,&result,MvTuple(Offset),MvTuple(MvString(LightDark).Text()));return result; }
MvRegion MvImage::DynThreshold(const MvImage& ThresholdImage, double Offset, const wchar_t* LightDark) const { MvRegion result;mv::DynThreshold(*this,ThresholdImage,&result,MvTuple(Offset),MvTuple(MvString(LightDark).Text()));return result; }
void PaintRegion(const MvObject& Region, const MvObject& Image, MvObject* ImageResult, const MvTuple& Grayval, const MvTuple& Type) { image_output(ImageResult,"PaintRegion",[&](::MvObject* h){return mv_paint_region(Region.NativeHandle(),Image.NativeHandle(),h,Grayval.ControlInput(),Type.ControlInput());}); }
MvImage MvImage::PaintRegion(const MvRegion& Region, const MvTuple& Grayval, const MvString& Type) const { MvImage result;mv::PaintRegion(Region,*this,&result,Grayval,MvTuple(MvString(Type).Text()));return result; }
MvImage MvImage::PaintRegion(const MvRegion& Region, double Grayval, const MvString& Type) const { MvImage result;mv::PaintRegion(Region,*this,&result,MvTuple(Grayval),MvTuple(MvString(Type).Text()));return result; }
MvImage MvImage::PaintRegion(const MvRegion& Region, double Grayval, const char* Type) const { MvImage result;mv::PaintRegion(Region,*this,&result,MvTuple(Grayval),MvTuple(MvString(Type).Text()));return result; }
MvImage MvImage::PaintRegion(const MvRegion& Region, double Grayval, const wchar_t* Type) const { MvImage result;mv::PaintRegion(Region,*this,&result,MvTuple(Grayval),MvTuple(MvString(Type).Text()));return result; }
MvImage MvRegion::PaintRegion(const MvImage& Image, const MvTuple& Grayval, const MvString& Type) const { return Image.PaintRegion(*this,Grayval,Type); }
MvImage MvRegion::PaintRegion(const MvImage& Image, double Grayval, const MvString& Type) const { return Image.PaintRegion(*this,Grayval,Type); }
MvImage MvRegion::PaintRegion(const MvImage& Image, double Grayval, const char* Type) const { return Image.PaintRegion(*this,Grayval,Type); }
MvImage MvRegion::PaintRegion(const MvImage& Image, double Grayval, const wchar_t* Type) const { return Image.PaintRegion(*this,Grayval,Type); }

void BinaryThreshold(const MvObject& Image,MvObject* Region,const MvTuple& Method,const MvTuple& LightDark,MvTuple* UsedThreshold) {
  cpp_api::require_output(Region,"BinaryThreshold");
  if(static_cast<void*>(Region)==static_cast<void*>(UsedThreshold))invalid_outputs("BinaryThreshold");
  cpp_api::TupleOutputs<1> controls("BinaryThreshold",{{UsedThreshold}});::MvObject raw=nullptr;
  cpp_api::check_status(mv_binary_threshold(Image.NativeHandle(),&raw,Method.ControlInput(),LightDark.ControlInput(),controls.Out(0)),"BinaryThreshold");
  auto region=detail::MvObjectAccess::Adopt(raw);detail::MvObjectAccess::CheckOutput(Region,region);
  controls.Commit();detail::MvObjectAccess::Commit(Region,std::move(region));
}
void AutoThreshold(const MvObject& Image,MvObject* Regions,const MvTuple& Sigma) {
  image_output(Regions,"AutoThreshold",[&](::MvObject* out){return mv_auto_threshold(Image.NativeHandle(),out,Sigma.ControlInput());});
}
void RotateImage(const MvObject& Image,MvObject* ImageRotate,const MvTuple& Phi,const MvTuple& Interpolation) {
  image_output(ImageRotate,"RotateImage",[&](::MvObject* out){return mv_rotate_image(Image.NativeHandle(),out,Phi.ControlInput(),Interpolation.ControlInput());});
}

MvRegion MvImage::BinaryThreshold(const MvString& Method, const MvString& LightDark, MvTuple* UsedThreshold) const {MvRegion result;mv::BinaryThreshold(*this,&result,MvTuple(Method),MvTuple(LightDark),UsedThreshold);return result;}
MvRegion MvImage::BinaryThreshold(const MvString& Method, const MvString& LightDark, std::int64_t* UsedThreshold) const {cpp_api::require_output(UsedThreshold,"BinaryThreshold");MvRegion result;MvTuple values;mv::BinaryThreshold(*this,&result,MvTuple(MvString(Method)),MvTuple(MvString(LightDark)),&values);if(values.Length()==0)invalid_scalar_output("BinaryThreshold");const auto first=values.I();*UsedThreshold=first;return result;}
MvRegion MvImage::BinaryThreshold(const char* Method, const char* LightDark, std::int64_t* UsedThreshold) const {cpp_api::require_output(UsedThreshold,"BinaryThreshold");MvRegion result;MvTuple values;mv::BinaryThreshold(*this,&result,MvTuple(MvString(Method)),MvTuple(MvString(LightDark)),&values);if(values.Length()==0)invalid_scalar_output("BinaryThreshold");const auto first=values.I();*UsedThreshold=first;return result;}
MvRegion MvImage::BinaryThreshold(const wchar_t* Method, const wchar_t* LightDark, std::int64_t* UsedThreshold) const {cpp_api::require_output(UsedThreshold,"BinaryThreshold");MvRegion result;MvTuple values;mv::BinaryThreshold(*this,&result,MvTuple(MvString(Method)),MvTuple(MvString(LightDark)),&values);if(values.Length()==0)invalid_scalar_output("BinaryThreshold");const auto first=values.I();*UsedThreshold=first;return result;}
MvRegion MvImage::AutoThreshold(const MvTuple& Sigma) const {MvRegion result;mv::AutoThreshold(*this,&result,Sigma);return result;}
MvRegion MvImage::AutoThreshold(double Sigma) const {MvRegion result;mv::AutoThreshold(*this,&result,MvTuple(Sigma));return result;}
MvImage MvImage::RotateImage(const MvTuple& Phi, const MvString& Interpolation) const {MvImage result;mv::RotateImage(*this,&result,Phi,MvTuple(MvString(Interpolation)));return result;}
MvImage MvImage::RotateImage(double Phi, const MvString& Interpolation) const {MvImage result;mv::RotateImage(*this,&result,MvTuple(Phi),MvTuple(MvString(Interpolation)));return result;}
MvImage MvImage::RotateImage(double Phi, const char* Interpolation) const {MvImage result;mv::RotateImage(*this,&result,MvTuple(Phi),MvTuple(MvString(Interpolation)));return result;}
MvImage MvImage::RotateImage(double Phi, const wchar_t* Interpolation) const {MvImage result;mv::RotateImage(*this,&result,MvTuple(Phi),MvTuple(MvString(Interpolation)));return result;}

namespace {
template<class Call> void color_output(MvObject* first,MvObject* second,MvObject* third,const char* op,Call&& call) {
  cpp_api::require_output(first,op);cpp_api::require_output(second,op);cpp_api::require_output(third,op);
  if(first==second||first==third||second==third)invalid_outputs(op);
  ::MvObject a=nullptr,b=nullptr,c=nullptr;cpp_api::check_status(call(&a,&b,&c),op);
  auto ra=detail::MvObjectAccess::Adopt(a),rb=detail::MvObjectAccess::Adopt(b),rc=detail::MvObjectAccess::Adopt(c);
  detail::MvObjectAccess::CheckOutput(first,ra);detail::MvObjectAccess::CheckOutput(second,rb);detail::MvObjectAccess::CheckOutput(third,rc);
  detail::MvObjectAccess::Commit(first,std::move(ra));detail::MvObjectAccess::Commit(second,std::move(rb));detail::MvObjectAccess::Commit(third,std::move(rc));
}
}
void TransFromRgb(const MvObject& ImageRed,const MvObject& ImageGreen,const MvObject& ImageBlue,MvObject* ImageResult1,MvObject* ImageResult2,MvObject* ImageResult3,const MvTuple& ColorSpace) {
  color_output(ImageResult1,ImageResult2,ImageResult3,"TransFromRgb",[&](::MvObject* a,::MvObject* b,::MvObject* c){return mv_trans_from_rgb(ImageRed.NativeHandle(),ImageGreen.NativeHandle(),ImageBlue.NativeHandle(),a,b,c,ColorSpace.ControlInput());});
}
void TransToRgb(const MvObject& ImageInput1,const MvObject& ImageInput2,const MvObject& ImageInput3,MvObject* ImageRed,MvObject* ImageGreen,MvObject* ImageBlue,const MvTuple& ColorSpace) {
  color_output(ImageRed,ImageGreen,ImageBlue,"TransToRgb",[&](::MvObject* a,::MvObject* b,::MvObject* c){return mv_trans_to_rgb(ImageInput1.NativeHandle(),ImageInput2.NativeHandle(),ImageInput3.NativeHandle(),a,b,c,ColorSpace.ControlInput());});
}
void DerivateGauss(const MvObject& Image,MvObject* DerivGauss,const MvTuple& Sigma,const MvTuple& Component) {
  image_output(DerivGauss,"DerivateGauss",[&](::MvObject* h){return mv_derivate_gauss(Image.NativeHandle(),h,Sigma.ControlInput(),Component.ControlInput());});
}
void LocalThreshold(const MvObject& Image,MvObject* Region,const MvTuple& Method,const MvTuple& LightDark,const MvTuple& GenParamName,const MvTuple& GenParamValue) {
  image_output(Region,"LocalThreshold",[&](::MvObject* h){return mv_local_threshold(Image.NativeHandle(),h,Method.ControlInput(),LightDark.ControlInput(),GenParamName.ControlInput(),GenParamValue.ControlInput());});
}
void WatershedsMarker(const MvObject& Image,const MvObject& Markers,MvObject* Basins) {
  image_output(Basins,"WatershedsMarker",[&](::MvObject* h){return mv_watersheds_marker(Image.NativeHandle(),Markers.NativeHandle(),h);});
}

MvImage MvImage::TransFromRgb(const MvImage& green,const MvImage& blue,MvImage* second,MvImage* third,const MvString& space) const {MvImage first;mv::TransFromRgb(*this,green,blue,&first,second,third,MvTuple(space));return first;}
MvImage MvImage::TransFromRgb(const MvImage& green,const MvImage& blue,MvImage* second,MvImage* third,const char* space) const {return TransFromRgb(green,blue,second,third,MvString(space));}
MvImage MvImage::TransFromRgb(const MvImage& green,const MvImage& blue,MvImage* second,MvImage* third,const wchar_t* space) const {return TransFromRgb(green,blue,second,third,MvString(space));}
MvImage MvImage::TransToRgb(const MvImage& second,const MvImage& third,MvImage* green,MvImage* blue,const MvString& space) const {MvImage red;mv::TransToRgb(*this,second,third,&red,green,blue,MvTuple(space));return red;}
MvImage MvImage::TransToRgb(const MvImage& second,const MvImage& third,MvImage* green,MvImage* blue,const char* space) const {return TransToRgb(second,third,green,blue,MvString(space));}
MvImage MvImage::TransToRgb(const MvImage& second,const MvImage& third,MvImage* green,MvImage* blue,const wchar_t* space) const {return TransToRgb(second,third,green,blue,MvString(space));}
MvImage MvImage::DerivateGauss(const MvTuple& sigma,const MvString& component) const {MvImage result;mv::DerivateGauss(*this,&result,sigma,MvTuple(component));return result;}
MvImage MvImage::DerivateGauss(double sigma,const MvString& component) const {return DerivateGauss(MvTuple(sigma),component);}
MvImage MvImage::DerivateGauss(double sigma,const char* component) const {return DerivateGauss(sigma,MvString(component));}
MvImage MvImage::DerivateGauss(double sigma,const wchar_t* component) const {return DerivateGauss(sigma,MvString(component));}
MvRegion MvImage::LocalThreshold(const MvString& method,const MvString& light,const MvTuple& names,const MvTuple& values) const {MvRegion result;mv::LocalThreshold(*this,&result,MvTuple(method),MvTuple(light),names,values);return result;}
MvRegion MvImage::LocalThreshold(const MvString& method,const MvString& light,const MvString& name,std::int64_t value) const {return LocalThreshold(method,light,MvTuple(name),MvTuple(value));}
MvRegion MvImage::LocalThreshold(const char* method,const char* light,const char* name,std::int64_t value) const {return LocalThreshold(MvString(method),MvString(light),MvString(name),value);}
MvRegion MvImage::LocalThreshold(const wchar_t* method,const wchar_t* light,const wchar_t* name,std::int64_t value) const {return LocalThreshold(MvString(method),MvString(light),MvString(name),value);}
MvRegion MvImage::WatershedsMarker(const MvRegion& markers) const {MvRegion result;mv::WatershedsMarker(*this,markers,&result);return result;}

void WriteImage(const MvObject& Image, const MvTuple& Format, const MvTuple& FillColor, const MvTuple& FileName) { cpp_api::check_status(mv_write_image(Image.NativeHandle(),Format.ControlInput(),FillColor.ControlInput(),FileName.ControlInput()),"WriteImage"); }
void MvImage::WriteImage(const MvString& Format, const MvTuple& FillColor, const MvTuple& FileName) const { mv::WriteImage(*this,MvTuple(MvString(Format)),FillColor,FileName); }
void MvImage::WriteImage(const MvString& Format, std::int64_t FillColor, const MvString& FileName) const { mv::WriteImage(*this,MvTuple(MvString(Format)),MvTuple(FillColor),MvTuple(MvString(FileName))); }
void MvImage::WriteImage(const char* Format, std::int64_t FillColor, const char* FileName) const { mv::WriteImage(*this,MvTuple(MvString(Format)),MvTuple(FillColor),MvTuple(MvString(FileName))); }
void MvImage::WriteImage(const wchar_t* Format, std::int64_t FillColor, const wchar_t* FileName) const { mv::WriteImage(*this,MvTuple(MvString(Format)),MvTuple(FillColor),MvTuple(MvString(FileName))); }
void AffineTransImage(const MvObject& Image, MvObject* ImageAffineTrans, const MvTuple& HomMat2D, const MvTuple& Interpolation, const MvTuple& AdaptImageSize) { image_output(ImageAffineTrans,"AffineTransImage",[&](::MvObject* h){return mv_affine_trans_image(Image.NativeHandle(),h,HomMat2D.ControlInput(),Interpolation.ControlInput(),AdaptImageSize.ControlInput());}); }
MvImage MvHomMat2D::AffineTransImage(const MvImage& Image, const MvString& Interpolation, const MvString& AdaptImageSize) const { MvImage result;mv::AffineTransImage(Image,&result,*this,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)));return result; }
MvImage MvHomMat2D::AffineTransImage(const MvImage& Image, const char* Interpolation, const char* AdaptImageSize) const { MvImage result;mv::AffineTransImage(Image,&result,*this,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)));return result; }
MvImage MvHomMat2D::AffineTransImage(const MvImage& Image, const wchar_t* Interpolation, const wchar_t* AdaptImageSize) const { MvImage result;mv::AffineTransImage(Image,&result,*this,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)));return result; }
MvImage MvImage::AffineTransImage(const MvHomMat2D& HomMat2D, const MvString& Interpolation, const MvString& AdaptImageSize) const { MvImage result;mv::AffineTransImage(*this,&result,HomMat2D,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)));return result; }
MvImage MvImage::AffineTransImage(const MvHomMat2D& HomMat2D, const char* Interpolation, const char* AdaptImageSize) const { MvImage result;mv::AffineTransImage(*this,&result,HomMat2D,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)));return result; }
MvImage MvImage::AffineTransImage(const MvHomMat2D& HomMat2D, const wchar_t* Interpolation, const wchar_t* AdaptImageSize) const { MvImage result;mv::AffineTransImage(*this,&result,HomMat2D,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)));return result; }
void MapImage(const MvObject& Image, const MvObject& Map, MvObject* ImageMapped) { image_output(ImageMapped,"MapImage",[&](::MvObject* h){return mv_map_image(Image.NativeHandle(),Map.NativeHandle(),h);}); }
MvImage MvImage::MapImage(const MvImage& Map) const { MvImage result;mv::MapImage(*this,Map,&result);return result; }
void PolarTransImageExt(const MvObject& Image, MvObject* PolarTransImage, const MvTuple& Row, const MvTuple& Column, const MvTuple& AngleStart, const MvTuple& AngleEnd, const MvTuple& RadiusStart, const MvTuple& RadiusEnd, const MvTuple& Width, const MvTuple& Height, const MvTuple& Interpolation) { image_output(PolarTransImage,"PolarTransImageExt",[&](::MvObject* h){return mv_polar_trans_image_ext(Image.NativeHandle(),h,Row.ControlInput(),Column.ControlInput(),AngleStart.ControlInput(),AngleEnd.ControlInput(),RadiusStart.ControlInput(),RadiusEnd.ControlInput(),Width.ControlInput(),Height.ControlInput(),Interpolation.ControlInput());}); }
MvImage MvImage::PolarTransImageExt(const MvTuple& Row, const MvTuple& Column, double AngleStart, double AngleEnd, const MvTuple& RadiusStart, const MvTuple& RadiusEnd, std::int64_t Width, std::int64_t Height, const MvString& Interpolation) const { MvImage result;mv::PolarTransImageExt(*this,&result,Row,Column,MvTuple(AngleStart),MvTuple(AngleEnd),RadiusStart,RadiusEnd,MvTuple(Width),MvTuple(Height),MvTuple(MvString(Interpolation)));return result; }
MvImage MvImage::PolarTransImageExt(double Row, double Column, double AngleStart, double AngleEnd, double RadiusStart, double RadiusEnd, std::int64_t Width, std::int64_t Height, const MvString& Interpolation) const { MvImage result;mv::PolarTransImageExt(*this,&result,MvTuple(Row),MvTuple(Column),MvTuple(AngleStart),MvTuple(AngleEnd),MvTuple(RadiusStart),MvTuple(RadiusEnd),MvTuple(Width),MvTuple(Height),MvTuple(MvString(Interpolation)));return result; }
MvImage MvImage::PolarTransImageExt(double Row, double Column, double AngleStart, double AngleEnd, double RadiusStart, double RadiusEnd, std::int64_t Width, std::int64_t Height, const char* Interpolation) const { MvImage result;mv::PolarTransImageExt(*this,&result,MvTuple(Row),MvTuple(Column),MvTuple(AngleStart),MvTuple(AngleEnd),MvTuple(RadiusStart),MvTuple(RadiusEnd),MvTuple(Width),MvTuple(Height),MvTuple(MvString(Interpolation)));return result; }
MvImage MvImage::PolarTransImageExt(double Row, double Column, double AngleStart, double AngleEnd, double RadiusStart, double RadiusEnd, std::int64_t Width, std::int64_t Height, const wchar_t* Interpolation) const { MvImage result;mv::PolarTransImageExt(*this,&result,MvTuple(Row),MvTuple(Column),MvTuple(AngleStart),MvTuple(AngleEnd),MvTuple(RadiusStart),MvTuple(RadiusEnd),MvTuple(Width),MvTuple(Height),MvTuple(MvString(Interpolation)));return result; }
void ProjectiveTransImage(const MvObject& Image, MvObject* TransImage, const MvTuple& HomMat2D, const MvTuple& Interpolation, const MvTuple& AdaptImageSize, const MvTuple& TransformDomain) { image_output(TransImage,"ProjectiveTransImage",[&](::MvObject* h){return mv_projective_trans_image(Image.NativeHandle(),h,HomMat2D.ControlInput(),Interpolation.ControlInput(),AdaptImageSize.ControlInput(),TransformDomain.ControlInput());}); }
MvImage MvHomMat2D::ProjectiveTransImage(const MvImage& Image, const MvString& Interpolation, const MvString& AdaptImageSize, const MvString& TransformDomain) const { MvImage result;mv::ProjectiveTransImage(Image,&result,*this,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)),MvTuple(MvString(TransformDomain)));return result; }
MvImage MvHomMat2D::ProjectiveTransImage(const MvImage& Image, const char* Interpolation, const char* AdaptImageSize, const char* TransformDomain) const { MvImage result;mv::ProjectiveTransImage(Image,&result,*this,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)),MvTuple(MvString(TransformDomain)));return result; }
MvImage MvHomMat2D::ProjectiveTransImage(const MvImage& Image, const wchar_t* Interpolation, const wchar_t* AdaptImageSize, const wchar_t* TransformDomain) const { MvImage result;mv::ProjectiveTransImage(Image,&result,*this,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)),MvTuple(MvString(TransformDomain)));return result; }
MvImage MvImage::ProjectiveTransImage(const MvHomMat2D& HomMat2D, const MvString& Interpolation, const MvString& AdaptImageSize, const MvString& TransformDomain) const { MvImage result;mv::ProjectiveTransImage(*this,&result,HomMat2D,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)),MvTuple(MvString(TransformDomain)));return result; }
MvImage MvImage::ProjectiveTransImage(const MvHomMat2D& HomMat2D, const char* Interpolation, const char* AdaptImageSize, const char* TransformDomain) const { MvImage result;mv::ProjectiveTransImage(*this,&result,HomMat2D,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)),MvTuple(MvString(TransformDomain)));return result; }
MvImage MvImage::ProjectiveTransImage(const MvHomMat2D& HomMat2D, const wchar_t* Interpolation, const wchar_t* AdaptImageSize, const wchar_t* TransformDomain) const { MvImage result;mv::ProjectiveTransImage(*this,&result,HomMat2D,MvTuple(MvString(Interpolation)),MvTuple(MvString(AdaptImageSize)),MvTuple(MvString(TransformDomain)));return result; }

} // namespace mv

namespace mv {
void GenImageConst(MvObject* Image, const MvTuple& Type, const MvTuple& Width, const MvTuple& Height) {
  image_output(Image, "GenImageConst", [&](::MvObject* raw) {
    return mv_gen_image_const(raw, Type.ControlInput(), Width.ControlInput(), Height.ControlInput());
  });
}
void GenImage1(MvObject* Image, const MvTuple& Type, const MvTuple& Width, const MvTuple& Height, const MvTuple& PixelPointer) {
  image_output(Image, "GenImage1", [&](::MvObject* raw) {
    return mv_gen_image1(raw, Type.ControlInput(), Width.ControlInput(), Height.ControlInput(), PixelPointer.ControlInput());
  });
}
void GenImage3(MvObject* ImageRGB, const MvTuple& Type, const MvTuple& Width, const MvTuple& Height, const MvTuple& PixelPointerRed, const MvTuple& PixelPointerGreen, const MvTuple& PixelPointerBlue) {
  image_output(ImageRGB, "GenImage3", [&](::MvObject* raw) {
    return mv_gen_image3(raw, Type.ControlInput(), Width.ControlInput(), Height.ControlInput(), PixelPointerRed.ControlInput(), PixelPointerGreen.ControlInput(), PixelPointerBlue.ControlInput());
  });
}
void GenImage1Extern(MvObject* Image, const MvTuple& Type, const MvTuple& Width, const MvTuple& Height, const MvTuple& PixelPointer, const MvTuple& ClearProc) {
  image_output(Image, "GenImage1Extern", [&](::MvObject* raw) {
    return mv_gen_image1_extern(raw, Type.ControlInput(), Width.ControlInput(), Height.ControlInput(), PixelPointer.ControlInput(), ClearProc.ControlInput());
  });
}
void GenImageInterleaved(MvObject* ImageRGB, const MvTuple& PixelPointer, const MvTuple& ColorFormat, const MvTuple& OriginalWidth, const MvTuple& OriginalHeight, const MvTuple& Alignment, const MvTuple& Type, const MvTuple& ImageWidth, const MvTuple& ImageHeight, const MvTuple& StartRow, const MvTuple& StartColumn, const MvTuple& BitsPerChannel, const MvTuple& BitShift) {
  image_output(ImageRGB, "GenImageInterleaved", [&](::MvObject* raw) {
    return mv_gen_image_interleaved(raw, PixelPointer.ControlInput(), ColorFormat.ControlInput(), OriginalWidth.ControlInput(), OriginalHeight.ControlInput(), Alignment.ControlInput(), Type.ControlInput(), ImageWidth.ControlInput(), ImageHeight.ControlInput(), StartRow.ControlInput(), StartColumn.ControlInput(), BitsPerChannel.ControlInput(), BitShift.ControlInput());
  });
}
void GetImagePointer1(const MvObject& Image, MvTuple* Pointer, MvTuple* Type, MvTuple* Width, MvTuple* Height) {
  cpp_api::TupleOutputs<4> out("GetImagePointer1", {Pointer, Type, Width, Height});
  cpp_api::check_status(mv_get_image_pointer1(Image.NativeHandle(), out.Out(0), out.Out(1), out.Out(2), out.Out(3)), "GetImagePointer1");
  out.Commit();
}
void GetImagePointer3(const MvObject& ImageRGB, MvTuple* PointerRed, MvTuple* PointerGreen, MvTuple* PointerBlue, MvTuple* Type, MvTuple* Width, MvTuple* Height) {
  cpp_api::TupleOutputs<6> out("GetImagePointer3", {PointerRed, PointerGreen, PointerBlue, Type, Width, Height});
  cpp_api::check_status(mv_get_image_pointer3(ImageRGB.NativeHandle(), out.Out(0), out.Out(1), out.Out(2), out.Out(3), out.Out(4), out.Out(5)), "GetImagePointer3");
  out.Commit();
}
void GetImageType(const MvObject& Image, MvTuple* Type) {
  cpp_api::TupleOutputs<1> out("GetImageType", {Type});
  cpp_api::check_status(mv_get_image_type(Image.NativeHandle(), out.Out(0)), "GetImageType");
  out.Commit();
}
void GetImageSize(const MvObject& Image, MvTuple* Width, MvTuple* Height) {
  cpp_api::TupleOutputs<2> out("GetImageSize", {Width, Height});
  cpp_api::check_status(mv_get_image_size(Image.NativeHandle(), out.Out(0), out.Out(1)), "GetImageSize");
  out.Commit();
}
void CountChannels(const MvObject& MultiChannelImage, MvTuple* Channels) {
  cpp_api::TupleOutputs<1> out("CountChannels", {Channels});
  cpp_api::check_status(mv_count_channels(MultiChannelImage.NativeHandle(), out.Out(0)), "CountChannels");
  out.Commit();
}
}

namespace mv {
void MvImage::GenImageConst(const MvTuple& Type, const MvTuple& Width, const MvTuple& Height) { mv::GenImageConst(this, Type, Width, Height); }
void MvImage::GenImageConst(const char* Type, std::int64_t Width, std::int64_t Height) { GenImageConst(MvTuple(Type), MvTuple(Width), MvTuple(Height)); }
void MvImage::GenImage1(const MvTuple& Type, const MvTuple& Width, const MvTuple& Height, const MvTuple& PixelPointer) { mv::GenImage1(this, Type, Width, Height, PixelPointer); }
void MvImage::GenImage1(const char* Type, std::int64_t Width, std::int64_t Height, const void* PixelPointer) { GenImage1(MvTuple(Type), MvTuple(Width), MvTuple(Height), MvTuple(static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(PixelPointer)))); }
void MvImage::GenImage3(const char* Type, std::int64_t Width, std::int64_t Height, const void* Red, const void* Green, const void* Blue) { mv::GenImage3(this, Type, Width, Height, static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(Red)), static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(Green)), static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(Blue))); }
void MvImage::GenImage1Extern(const char* Type, std::int64_t Width, std::int64_t Height, void* PixelPointer, void* ClearProc) { mv::GenImage1Extern(this, Type, Width, Height, static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(PixelPointer)), static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(ClearProc))); }
MvTuple MvImage::GetImageType() const { MvTuple result; mv::GetImageType(*this, &result); return result; }
MvTuple MvImage::GetImagePointer1(MvTuple* Type, MvTuple* Width, MvTuple* Height) const { MvTuple result; mv::GetImagePointer1(*this, &result, Type, Width, Height); return result; }
}
