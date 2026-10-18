#include <mvkit/c/mv_image.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>

#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "structured_error.hpp"
#include "tuple_input.hpp"
#include "image_controls.hpp"
#include <imageproc/operators.hpp>
#include <cstring>
#include <sstream>
#include <core/exception.hpp>
#include <imageio/builtins.hpp>
#include <imageio/codec.hpp>
#include <imageio/stream.hpp>

namespace {
void require_path(const char* path, const char* where) {
  if (path == nullptr || *path == '\0') {
    throw mv::c_api::Exception(mv::core::Status::InvalidArgument, where, "path must not be null or empty");
  }
}

mv::imageio::ImageFormat format_from_path(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (extension == ".bmp" || extension == ".dib") return mv::imageio::ImageFormat::Bmp;
  if (extension == ".jpg" || extension == ".jpeg" || extension == ".jpe") return mv::imageio::ImageFormat::Jpeg;
  if (extension == ".png") return mv::imageio::ImageFormat::Png;
  if (extension == ".tif" || extension == ".tiff") return mv::imageio::ImageFormat::Tiff;
  throw mv::c_api::Exception(mv::core::Status::Unsupported, "mv_write_image", "unsupported output file extension");
}
}

extern "C" ::MvStatus mv_read_image(::MvObject* image, ::MvTuple filenames) {
  return mv::c_api::translate([&] {
    mv::c_api::require_output(image,"read_image","Image");
    const mv::c_api::TupleInput files(filenames,"read_image","FileName");
    if (!files.size()) files.fail(mv::core::ErrorCode::InvalidCardinality,-1,"at least one file name required");
    // 访问文件或提交输出前验证整批输入。
    for (std::size_t i=0;i<files.size();++i) {
      const auto path=files.string(i);
      if (path.empty() || path.find('\0')!=std::string_view::npos)
        files.fail(mv::core::ErrorCode::InvalidValue,i,"non-empty file name without NUL required");
    }
    std::vector<mv::core::Image> values;values.reserve(files.size());
    for (std::size_t i=0;i<files.size();++i) {
      mv::imageio::FileInputStream stream{std::filesystem::u8path(files.string(i))};
      if (!stream.good())
        throw mv::core::Exception(mv::core::Status::IoError,mv::core::ErrorCode::IoFailure,
            "read_image","FileName",i,"failed to open input image file");
      values.push_back(mv::imageio::default_image_io_service().read(stream));
    }
    auto created = mv::c_api::make_iconic(mv::core::make_image_tuple(std::move(values)));
    *image = created.release();
  });
}

namespace {
std::filesystem::path output_path(const mv::c_api::TupleInput& input) {
  const auto text=mv::c_api::image_string(input);
  if(text.empty()||text.find('\0')!=std::string_view::npos)input.fail(mv::core::ErrorCode::InvalidValue,0,"nonempty filename without NUL required");
  return std::filesystem::u8path(text);
}
void write_encoded(const std::filesystem::path& path,mv::imageio::ImageFormat format,
                   const mv::core::Image& image,const mv::imageio::EncodeOptions& options={}) {
  mv::imageio::MemoryOutputStream encoded;
  mv::imageio::default_image_io_service().write(encoded,format,image,options);
  mv::imageio::FileOutputStream stream(path);
  const auto bytes=encoded.data();
  if(!stream.good()||stream.write(bytes.data(),bytes.size())!=bytes.size())
    throw mv::core::Exception(mv::core::Status::IoError,mv::core::ErrorCode::IoFailure,"write_image","FileName",0,"failed to write output image file");
}
}

extern "C" ::MvStatus mv_write_image(::MvObject Image,::MvTuple Format,::MvTuple FillColor,::MvTuple FileName) {
  return mv::c_api::translate([&]{
    constexpr auto op="write_image";mv::c_api::require_iconic(Image,op,"Image");
    Image->value.require_kind(mv::core::IconicKind::Image,op);
    if(Image->value.empty())return;
    const mv::c_api::TupleInput formats(Format,op,"Format"),fills(FillColor,op,"FillColor"),files(FileName,op,"FileName");
    const auto text=mv::c_api::image_string(formats);auto path=output_path(files);
    const auto fill=mv::c_api::image_scalar_integer(fills);
    if(fill<0||fill>0xffffff)fills.fail(mv::core::ErrorCode::InvalidValue,0,"FillColor must be within [0,0xffffff]");
    mv::imageio::ImageFormat format;std::string suffix;mv::imageio::EncodeOptions options;
    if(text=="bmp"){format=mv::imageio::ImageFormat::Bmp;suffix=".bmp";}
    else if(text=="png"){format=mv::imageio::ImageFormat::Png;suffix=".png";}
    else if(text=="tiff"||text=="tiff none"){format=mv::imageio::ImageFormat::Tiff;suffix=".tif";}
    else if(text=="jpeg"||text.starts_with("jpeg ")){
      format=mv::imageio::ImageFormat::Jpeg;suffix=".jpg";options.jpeg_quality=75;
      if(text!="jpeg") {
        std::istringstream spec(std::string{text.substr(5)});int quality;char rest;
        if(!(spec>>quality)||quality<0||quality>100||(spec>>rest))formats.fail(mv::core::ErrorCode::InvalidValue,0,"jpeg quality must be an integer in [0,100]");
        options.jpeg_quality=quality;
      }
    } else throw mv::core::Exception(mv::core::Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"Format",0,"supported formats are bmp, png, tiff, tiff none and jpeg [quality]");
    if(Image->value.size()!=1)throw mv::core::Exception(mv::core::Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"Image",-1,"multi-page image writing is not implemented");
    auto extension=path.extension().string();
    std::transform(extension.begin(),extension.end(),extension.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
    const bool matches=extension==suffix||(format==mv::imageio::ImageFormat::Tiff&&extension==".tiff")||
                       (format==mv::imageio::ImageFormat::Jpeg&&extension==".jpeg");
    if(!matches)path+=suffix;
    auto image=Image->value.at(0).as_image();
    if(image.has_reduced_domain()) {
      if(format==mv::imageio::ImageFormat::Png||format==mv::imageio::ImageFormat::Tiff)
        throw mv::core::Exception(mv::core::Status::Unsupported,mv::core::ErrorCode::UnsupportedValue,op,"Image",-1,"PNG/TIFF domain-mask serialization is not implemented");
      image=image.clone();const auto outside=image.full_domain().domain().difference(image.domain());
      for(std::size_t ch=0;ch<image.channel_count();++ch) {
        if(image.pixel_type(ch)!=mv::core::PixelType::UInt8)throw mv::core::Exception(mv::core::Status::TypeMismatch,mv::core::ErrorCode::InvalidType,op,"Image",-1,"bmp/jpeg require byte images");
        const auto value=static_cast<std::uint8_t>(image.channel_count()==3?fill>>(8*(2-ch)):fill);
        auto* data=static_cast<std::byte*>(image.mutable_data(ch));
        for(const auto& run:outside.runs())std::memset(data+run.row*image.stride(ch)+run.col_begin,value,std::int64_t(run.col_end)-std::int64_t(run.col_begin)+1);
      }
      image=image.full_domain();
    }
    write_encoded(path,format,image,options);
  });
}
