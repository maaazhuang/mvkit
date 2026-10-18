#include <filesystem>
#include <shape_matching/operators.hpp>
#include <core/exception.hpp>
#include <core/model_io.hpp>
#include <core/model_text.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>

namespace mv::shape_matching {
namespace {
constexpr std::size_t max_file = 256U * 1024U * 1024U;
constexpr std::size_t max_points = 1000000;
[[noreturn]] void fail(core::Status status, const char* message) {
  throw core::Exception(status, core::default_error_code(status), "shape_model_persistence", "path", -1, message);
}
uint64_t checksum(std::string_view text) {
  uint64_t hash=14695981039346656037ULL;
  for(unsigned char c:text) { hash^=c; hash*=1099511628211ULL; }
  return hash;
}
template<class T,class Input> T read(Input& in) {
  T value{};
  if(!(in>>value)) fail(core::Status::DecodeError,"truncated or invalid model field");
  return value;
}
double real(core::model_io::TextInput& in) {
  auto v=read<double>(in);
  if(!std::isfinite(v))fail(core::Status::DecodeError,"non-finite model field");
  return v;
}
std::size_t count(core::model_io::TextInput& in, std::size_t limit=max_points) {
  const auto n=read<int64_t>(in);
  if(n<0 || static_cast<uint64_t>(n)>limit)fail(core::Status::LimitExceeded,"model count exceeds limit");
  return static_cast<size_t>(n);
}
void points_out(core::model_io::TextOutput& out, std::span<const ShapeModelPoint> points) {
  if(points.size()>max_points)fail(core::Status::LimitExceeded,"model point count exceeds limit");
  out<<points.size()<<'\n';
  for(const auto& p:points) out<<p.row<<' '<<p.column<<' '<<p.normal_row<<' '<<p.normal_column<<' '<<p.response<<'\n';
}
void points_in(core::model_io::TextInput& in, std::pmr::vector<ShapeModelPoint>& points) {
  const auto n=count(in); points.reserve(n);
  for(size_t i=0;i<n;++i) points.push_back({real(in),real(in),real(in),real(in),real(in)});
}
}
struct ShapeModelPersistence {
  static std::string encode(const ShapeModel& m) {
    if(m.levels_.size()>30)fail(core::Status::LimitExceeded,"too many pyramid levels");
    if(m.empty())fail(core::Status::InvalidState,"cannot save an empty model");
    core::model_io::TextOutput s;
    const auto metric = m.metric_name_ == "ignore_color_polarity" ? 3 : static_cast<int>(m.metric_);
    s<<static_cast<int>(m.kind_)<<' '<<metric<<'\n';
    s<<m.default_origin_row_<<' '<<m.default_origin_column_<<' '<<m.origin_row_<<' '<<m.origin_column_<<'\n';
    s<<m.angle_start_<<' '<<m.angle_extent_<<' '<<m.angle_step_<<' '<<m.scale_min_<<' '<<m.scale_max_<<' '<<m.scale_step_<<'\n';
    s<<m.min_contrast_<<' '<<m.timeout_ms_<<' '<<m.border_shape_models_<<' '<<m.model_cache_enabled_<<'\n';
    points_out(s,m.refinement_points_); s<<m.levels_.size()<<'\n';
    for(const auto& l:m.levels_) {
      s<<l.level<<' '<<l.pyramid_scale<<' '<<l.radius<<'\n'; points_out(s,l.points);
      if(l.contours.contour_count()>max_points)fail(core::Status::LimitExceeded,"too many contours");
      s<<l.contours.contour_count()<<'\n';
      for(size_t i=0;i<l.contours.contour_count();++i) {
        const auto& c=l.contours.contour(i); if(c.size()>max_points)fail(core::Status::LimitExceeded,"contour exceeds point limit"); s<<c.closed()<<' '<<static_cast<int>(c.approximation())<<' '<<c.size()<<'\n';
        for(const auto& p:c.points())s<<p.row<<' '<<p.col<<'\n';
        for(auto attr:{core::XldPointAttribute::Response,core::XldPointAttribute::EdgeDirection,core::XldPointAttribute::NormalAngle}) {
          const auto values=c.point_attribute(attr); s<<values.size()<<'\n';
          for(double v:values)s<<v<<' '; s<<'\n';
        }
      }
    }
    auto text=std::move(s).str(); if(text.size()>max_file)fail(core::Status::LimitExceeded,"model file exceeds 256 MiB");
    return text;
  }
  static ShapeModel decode(std::string_view text, bool xld, bool color_alias = false) {
    core::model_io::TextInput s(text); ShapeModel m;
    m.xld_model_=xld;
    const auto kind=count(s,1), metric=count(s,color_alias?3:2);
    m.kind_=static_cast<ShapeModelKind>(kind); m.metric_=static_cast<ShapeMetric>(metric==3?2:metric);
    const char* names[]{"use_polarity","ignore_global_polarity","ignore_local_polarity","ignore_color_polarity"}; m.metric_name_=names[metric];
    m.default_origin_row_=real(s); m.default_origin_column_=real(s); m.origin_row_=real(s); m.origin_column_=real(s);
    m.angle_start_=real(s); m.angle_extent_=real(s); m.angle_step_=real(s);
    m.scale_min_=real(s); m.scale_max_=real(s); m.scale_step_=real(s);
    m.min_contrast_=real(s); m.timeout_ms_=real(s); m.border_shape_models_=count(s,1)!=0; m.model_cache_enabled_=count(s,1)!=0;
    if(m.angle_extent_<0 || m.angle_step_<=0 || m.scale_min_<=0 || m.scale_max_<m.scale_min_ || m.scale_step_<0 || m.min_contrast_<0)
      fail(core::Status::DecodeError,"invalid model parameters");
    points_in(s,m.refinement_points_); const auto n=count(s,30);
    if(n==0)fail(core::Status::DecodeError,"model has no levels");
    for(size_t i=0;i<n;++i) {
      ShapeModelLevel l; l.level=static_cast<int>(count(s,29)); l.pyramid_scale=real(s); l.radius=real(s);
      if(l.level!=static_cast<int>(i) || l.pyramid_scale!=std::ldexp(1.0,static_cast<int>(i)) || l.radius<0)fail(core::Status::DecodeError,"invalid pyramid level");
      points_in(s,l.points); if(l.points.empty())fail(core::Status::DecodeError,"empty model level");
      const auto nc=count(s);
      for(size_t j=0;j<nc;++j) {
        const bool closed=count(s,1)!=0; const auto approx=static_cast<core::ContourApproximation>(count(s,3)); const auto np=count(s);
        std::vector<core::Point2d> points; points.reserve(np);
        for(size_t k=0;k<np;++k)points.push_back({real(s),real(s)});
        std::vector<double> attrs[3];
        for(auto& a:attrs) { const auto na=count(s); if(na!=0 && na!=np)fail(core::Status::DecodeError,"invalid contour attribute count"); a.reserve(na); for(size_t k=0;k<na;++k)a.push_back(real(s)); }
        l.contours.append(core::XldContour::from_points_and_attributes(points,closed,approx,attrs[0],attrs[1],attrs[2]));
      }
      m.levels_.push_back(std::move(l));
    }
    s>>std::ws; if(!s.eof())fail(core::Status::DecodeError,"trailing model data");
    m.search_cache_=make_shape_model_search_cache(); return m;
  }
};
std::string serialize_model(const ShapeModel& model) {
  auto body=ShapeModelPersistence::encode(model);
  // 旧格式继续读写；第三版用于保留图像模型的颜色极性别名。
  const auto magic=model.metric_name()=="ignore_color_polarity"?"MVKIT_SHAPE 3\n":
      model.is_xld_model()?"MVKIT_SHAPE 2\n":"MVKIT_SHAPE 1\n";
  return core::model_io::enclose_text(magic,checksum(body),std::move(body));
}
ShapeModel deserialize_model(std::string_view bytes) {
  if(bytes.size()>max_file+128)fail(core::Status::LimitExceeded,"model file exceeds limit");
  core::model_io::TextViewInput in(bytes);
  std::string magic; std::getline(in,magic);
  if(magic!="MVKIT_SHAPE 1" && magic!="MVKIT_SHAPE 2" && magic!="MVKIT_SHAPE 3") {
    if(magic.rfind("MVKIT_SHAPE ",0)==0)fail(core::Status::Unsupported,"unsupported model version");
    fail(core::Status::DecodeError,"invalid model signature");
  }
  uint64_t expected=read<uint64_t>(in); if(in.get()!='\n')fail(core::Status::DecodeError,"invalid checksum header");
  const auto body=in.remaining();
  if(checksum(body)!=expected)fail(core::Status::DecodeError,"model checksum mismatch");
  return ShapeModelPersistence::decode(body,magic=="MVKIT_SHAPE 2",magic=="MVKIT_SHAPE 3");
}
void write_model(const ShapeModel& model,std::string_view path) {
  core::model_io::write_file(path,serialize_model(model),"write_shape_model");
}
ShapeModel read_model(std::string_view path) {
  return deserialize_model(core::model_io::read_file(path,max_file+128,"read_shape_model"));
}
} // namespace mv::shape_matching
