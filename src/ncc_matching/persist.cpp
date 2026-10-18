#include "model.hpp"
#include <core/exception.hpp>
#include <core/model_io.hpp>
#include <core/model_text.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>

namespace mv::ncc_matching {
namespace {
constexpr std::size_t max_file=64*1024*1024,max_pixels=1048576;
[[noreturn]] void fail(core::Status status,const char* message) {
  throw core::Exception(status,core::default_error_code(status),"ncc_model_persistence","path",-1,message);
}
std::uint64_t checksum(std::string_view text) {
  std::uint64_t hash=14695981039346656037ULL;
  for(unsigned char c:text){hash^=c;hash*=1099511628211ULL;}return hash;
}
template<class T,class Input> T field(Input& input) {
  T value{};if(!(input>>value))fail(core::Status::DecodeError,"invalid or truncated model field");return value;
}
double real(core::model_io::TextInput& input) {
  const auto value=field<double>(input);if(!std::isfinite(value))fail(core::Status::DecodeError,"non-finite model field");return value;
}
std::size_t count(core::model_io::TextInput& input,std::size_t limit) {
  const auto value=field<std::int64_t>(input);
  if(value<0 || std::uint64_t(value)>limit)fail(core::Status::LimitExceeded,"model count exceeds budget");return std::size_t(value);
}
}
std::string serialize(const Model& model) {
  if(model.levels.empty())fail(core::Status::InvalidState,"cannot save empty model");
  const auto& level=model.levels.front();const auto& o=model.options;
  core::model_io::TextOutput body;
  body<<o.levels<<' '<<o.ignore_global_polarity<<' '<<o.angle_start<<' '<<o.angle_extent<<' '<<o.angle_step<<'\n';
  body<<model.origin_row<<' '<<model.origin_column<<' '<<level.width<<' '<<level.height<<'\n';
  body<<level.origin_row<<' '<<level.origin_column<<'\n';
  for(std::size_t i=0;i<level.gray.size();++i)body<<int(level.mask[i])<<' '<<level.gray[i]<<'\n';
  auto text=std::move(body).str();if(text.size()>max_file)fail(core::Status::LimitExceeded,"serialized model exceeds file budget");
  return core::model_io::enclose_text("MVKIT_NCC 1\n",checksum(text),std::move(text));
}
Model deserialize(std::string_view bytes) {
  if(bytes.size()>max_file+128)fail(core::Status::LimitExceeded,"model file exceeds budget");
  core::model_io::TextViewInput input(bytes);
  std::string magic;std::getline(input,magic);
  if(magic!="MVKIT_NCC 1")fail(magic.starts_with("MVKIT_NCC ")?core::Status::Unsupported:core::Status::DecodeError,"invalid model signature or version");
  const auto expected=field<std::uint64_t>(input);if(input.get()!='\n')fail(core::Status::DecodeError,"invalid checksum header");
  const auto text=input.remaining();
  if(checksum(text)!=expected)fail(core::Status::DecodeError,"NCC model checksum mismatch");
  core::model_io::TextInput body(text);CreateOptions options;
  options.levels=int(count(body,16));options.ignore_global_polarity=count(body,1)!=0;
  options.angle_start=real(body);options.angle_extent=real(body);options.angle_step=real(body);
  if(options.angle_extent<0 || options.angle_extent>6.283185307179586 || options.angle_step<=0 || !std::isfinite(options.angle_start+options.angle_extent))fail(core::Status::DecodeError,"invalid trained angle interval");
  const auto origin_row=real(body),origin_column=real(body);Level first;
  first.width=count(body,max_pixels);first.height=count(body,max_pixels);
  if(!first.width || !first.height || first.height>max_pixels/first.width)fail(core::Status::LimitExceeded,"invalid model dimensions");
  first.origin_row=real(body);first.origin_column=real(body);
  first.gray.resize(first.width*first.height);first.mask.resize(first.gray.size());
  long double row_sum=0,column_sum=0;std::size_t samples=0;
  for(std::size_t i=0;i<first.gray.size();++i) {
    first.mask[i]=std::uint8_t(count(body,1));first.gray[i]=real(body);
    if(first.mask[i]){row_sum+=i/first.width;column_sum+=i%first.width;++samples;}
  }
  if(samples<4 || std::abs(first.origin_row-double(row_sum/samples))>1e-10 ||
     std::abs(first.origin_column-double(column_sum/samples))>1e-10 ||
     origin_row<first.origin_row || origin_column<first.origin_column ||
     std::abs((origin_row-first.origin_row)-std::round(origin_row-first.origin_row))>1e-8 ||
     std::abs((origin_column-first.origin_column)-std::round(origin_column-first.origin_column))>1e-8)
    fail(core::Status::DecodeError,"invalid template domain centroid");
  body>>std::ws;if(!body.eof())fail(core::Status::DecodeError,"trailing model data");
  try{return rebuild(std::move(first),options,origin_row,origin_column);}
  catch(const core::Exception& error) {
    fail(error.status()==core::Status::LimitExceeded?error.status():core::Status::DecodeError,"invalid prepared NCC model");
  }
}
void write(const Model& model,std::string_view path) {
  core::model_io::write_file(path,serialize(model),"write_ncc_model");
}
Model read(std::string_view path) {
  return deserialize(core::model_io::read_file(path,max_file+128,"read_ncc_model"));
}
} // namespace mv::ncc_matching
