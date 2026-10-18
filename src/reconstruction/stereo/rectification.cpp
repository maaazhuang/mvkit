#include <reconstruction/stereo/rectification.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#include <calibration/camera_model.hpp>
#include <calibration/geometry3d.hpp>
#include <core/exception.hpp>

namespace mv::reconstruction {
namespace {

double dot(const Point3d& a,const Point3d& b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Point3d sub(const Point3d&a,const Point3d&b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Point3d mul(const Point3d&a,double s){return {a.x*s,a.y*s,a.z*s};}
Point3d cross(const Point3d&a,const Point3d&b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double norm(const Point3d&a){return std::sqrt(dot(a,a));}
Point3d normalize(const Point3d&a){const double n=norm(a);if(n<1e-14)throw core::Exception(core::Status::InvalidState,"gen_binocular_rectification_map","degenerate stereo geometry");return mul(a,1.0/n);}

struct PerspectiveInfo { double focus{},sx{},sy{},cx{},cy{}; std::int64_t width{},height{}; };
PerspectiveInfo info(const calibration::CameraParameters& p){
  if(!calibration::is_perspective(p)) throw core::Exception(core::Status::Unsupported,"gen_binocular_rectification_map","automatic rectification supports perspective cameras only");
  const auto& c=calibration::common_parameters(p);
  double focus=std::visit([](const auto& v)->double{
    using T=std::decay_t<decltype(v)>;
    if constexpr(std::is_same_v<T,calibration::AreaScanDivisionParameters>||std::is_same_v<T,calibration::AreaScanPolynomialParameters>) return v.focus;
    else return 0.0;
  },p);
  return {focus,c.sx,c.sy,c.cx,c.cy,c.image_width,c.image_height};
}

struct RectifiedBounds {
  double min_x{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
};

void include_rectified_boundary(RectifiedBounds& bounds,
                                const calibration::CameraParameters& camera,
                                std::int64_t width,
                                std::int64_t height,
                                const auto& to_rect) {
  const auto include = [&](double row, double col) {
    const auto ray = calibration::image_point_to_ray(camera, {row, col});
    const auto direction = to_rect(ray.direction);
    if (direction.z <= 1e-12) return;
    const double x = direction.x / direction.z;
    const double y = direction.y / direction.z;
    bounds.min_x = std::min(bounds.min_x, x);
    bounds.max_x = std::max(bounds.max_x, x);
    bounds.min_y = std::min(bounds.min_y, y);
    bounds.max_y = std::max(bounds.max_y, y);
  };
  constexpr std::int64_t subdivisions=8;
  const auto horizontal_steps=(width-1)*subdivisions;
  const auto vertical_steps=(height-1)*subdivisions;
  for (std::int64_t step = 0; step <= horizontal_steps; ++step) {
    const double col=static_cast<double>(step)/static_cast<double>(subdivisions);
    include(0.0, col);
    include(static_cast<double>(height - 1), col);
  }
  for (std::int64_t step = 1; step < vertical_steps; ++step) {
    const double row=static_cast<double>(step)/static_cast<double>(subdivisions);
    include(row, 0.0);
    include(row, static_cast<double>(width - 1));
  }
}

Point3d to_rect_direction(const StereoRectification& rect, std::int64_t camera_index, Point3d dir) {
  if(camera_index==1) dir=calibration::rotate_point(rect.original_setup().device_1_in_device_0_pose().rotation,dir);
  const auto inv=calibration::inverse_pose(rect.rectified_to_original_device0_pose());
  return calibration::rotate_point(inv.rotation,dir);
}

Point3d from_rect_direction(const StereoRectification& rect, std::int64_t camera_index, Point3d dir) {
  Point3d d0=calibration::rotate_point(rect.rectified_to_original_device0_pose().rotation,dir);
  if(camera_index==0) return d0;
  const auto inv=calibration::inverse_pose(rect.original_setup().device_1_in_device_0_pose());
  return calibration::rotate_point(inv.rotation,d0);
}

template <class T>
T bilerp_scalar(const core::Image& src,double r,double c,std::size_t channel){
  if(r<0.0||c<0.0||r>static_cast<double>(src.height()-1)||c>static_cast<double>(src.width()-1)) return T{};
  const auto r0=static_cast<std::size_t>(std::floor(r)); const auto c0=static_cast<std::size_t>(std::floor(c));
  const auto r1=std::min(r0+1,src.height()-1); const auto c1=std::min(c0+1,src.width()-1);
  const double fr=r-static_cast<double>(r0), fc=c-static_cast<double>(c0);
  auto value=[&](std::size_t rr,std::size_t cc){const auto* row=reinterpret_cast<const T*>(static_cast<const std::byte*>(src.data(channel))+rr*src.stride(channel));return static_cast<double>(row[cc]);};
  const double v=(1-fr)*((1-fc)*value(r0,c0)+fc*value(r0,c1))+fr*((1-fc)*value(r1,c0)+fc*value(r1,c1));
  if constexpr(std::is_floating_point_v<T>) return static_cast<T>(v);
  else return static_cast<T>(std::clamp(std::llround(v),static_cast<long long>(std::numeric_limits<T>::min()),static_cast<long long>(std::numeric_limits<T>::max())));
}

} // namespace

std::size_t StereoRectification::checked_index(std::int64_t index){if(index<0||index>1)throw core::Exception(core::Status::InvalidArgument,"StereoRectification","camera index must be 0 or 1");return static_cast<std::size_t>(index);}
const calibration::CameraParameters& StereoRectification::camera(std::int64_t index) const{return rectified_cameras_[checked_index(index)];}
const core::Image& StereoRectification::map_row(std::int64_t index) const{return map_row_[checked_index(index)];}
const core::Image& StereoRectification::map_col(std::int64_t index) const{return map_col_[checked_index(index)];}

StereoRectification gen_binocular_rectification_map(const CameraSetupModel& setup){
  if(setup.kind()!=CameraSetupKind::CameraCamera||!setup.complete()) throw core::Exception(core::Status::InvalidState,"gen_binocular_rectification_map","complete camera-camera setup required");
  const auto i0=info(setup.camera(0)), i1=info(setup.camera(1));
  const auto rel=setup.device_1_in_device_0_pose();
  Point3d x=normalize(rel.translation);
  const Point3d z0{0,0,1};
  const Point3d z1=calibration::rotate_point(rel.rotation,{0,0,1});
  Point3d zguess=normalize({z0.x+z1.x,z0.y+z1.y,z0.z+z1.z});
  Point3d z=normalize(sub(zguess,mul(x,dot(zguess,x))));
  Point3d y=normalize(cross(z,x));
  z=normalize(cross(x,y));
  const std::array<double,9> r{x.x,y.x,z.x,x.y,y.y,z.y,x.z,y.z,z.z};
  Pose3d rect_to0{{0,0,0},calibration::quaternion_from_rotation_matrix(r)};
  const double baseline=norm(rel.translation);
  Pose3d rect_rel{{baseline,0,0},{1,0,0,0}};
  const double pixel_size=std::min(i0.sy,i1.sy);
  const auto center_ray0=calibration::image_point_to_ray(
      setup.camera(0),{0.5*static_cast<double>(i0.height),0.5*static_cast<double>(i0.width)});
  const auto center_ray1=calibration::image_point_to_ray(
      setup.camera(1),{0.5*static_cast<double>(i1.height),0.5*static_cast<double>(i1.width)});
  const Point3d center_point0=mul(center_ray0.direction,i0.focus);
  const Point3d center_point1_in0=calibration::rotate_point(rel.rotation,mul(center_ray1.direction,i1.focus));
  const double focus=0.5*(dot(center_point0,z)+dot(center_point1_in0,z));
  const auto rect_to0_inverse=calibration::inverse_pose(rect_to0);
  RectifiedBounds bounds0,bounds1;
  include_rectified_boundary(bounds0,setup.camera(0),i0.width,i0.height,[&](Point3d direction){
    return calibration::rotate_point(rect_to0_inverse.rotation,direction);
  });
  include_rectified_boundary(bounds1,setup.camera(1),i1.width,i1.height,[&](Point3d direction){
    direction=calibration::rotate_point(rel.rotation,direction);
    return calibration::rotate_point(rect_to0_inverse.rotation,direction);
  });
  const double scale=focus/pixel_size;
  const double min_y=scale*std::max(bounds0.min_y,bounds1.min_y);
  const double max_y=scale*std::min(bounds0.max_y,bounds1.max_y);
  if (!(max_y > min_y)) throw core::Exception(core::Status::InvalidState,"gen_binocular_rectification_map","rectified camera views have no common vertical field of view");
  const double cy=-min_y;
  const auto height=static_cast<std::int64_t>(std::ceil((max_y-min_y)-1e-9))+1;
  const auto make_rect_camera=[&](const RectifiedBounds& bounds){
    calibration::AreaScanDivisionParameters rp;
    const double min_x=scale*bounds.min_x;
    const double max_x=scale*bounds.max_x;
    rp.focus=focus;rp.kappa=0.0;rp.sx=pixel_size;rp.sy=pixel_size;
    rp.cx=-min_x;rp.cy=cy;
    rp.image_width=static_cast<std::int64_t>(std::floor((max_x-min_x)+1e-9))+1;
    rp.image_height=height;
    return calibration::CameraParameters{rp};
  };
  calibration::CameraParameters rc0=make_rect_camera(bounds0),rc1=make_rect_camera(bounds1);
  const auto width0=calibration::common_parameters(rc0).image_width;
  const auto width1=calibration::common_parameters(rc1).image_width;
  std::array<core::Image,2> mr{core::Image::create(width0,height,core::PixelType::Float32),core::Image::create(width1,height,core::PixelType::Float32)};
  std::array<core::Image,2> mc{core::Image::create(width0,height,core::PixelType::Float32),core::Image::create(width1,height,core::PixelType::Float32)};
  StereoRectification rect(rc0,rc1,rect_to0,rect_rel,setup,mr,mc);
  for(std::int64_t cam=0;cam<2;++cam){
    const auto camera_width=calibration::common_parameters(rect.camera(cam)).image_width;
    for(std::size_t row=0;row<static_cast<std::size_t>(height);++row){
      auto* rr=reinterpret_cast<float*>(static_cast<std::byte*>(const_cast<core::Image&>(rect.map_row(cam)).mutable_data())+row*rect.map_row(cam).stride());
      auto* cc=reinterpret_cast<float*>(static_cast<std::byte*>(const_cast<core::Image&>(rect.map_col(cam)).mutable_data())+row*rect.map_col(cam).stride());
      for(std::size_t col=0;col<static_cast<std::size_t>(camera_width);++col){
        const auto ray=calibration::image_point_to_ray(rect.camera(cam),{static_cast<double>(row),static_cast<double>(col)});
        const auto orig_dir=from_rect_direction(rect,cam,ray.direction);
        try{const auto raw=calibration::project_camera_point(setup.camera(cam),orig_dir);rr[col]=static_cast<float>(raw.row);cc[col]=static_cast<float>(raw.col);}catch(const core::Exception&){rr[col]=std::numeric_limits<float>::quiet_NaN();cc[col]=std::numeric_limits<float>::quiet_NaN();}
      }
    }
  }
  return rect;
}

Point2d rectify_image_point(const StereoRectification& rect,std::int64_t camera_index,const Point2d& raw_point){
  const auto raw=calibration::image_point_to_ray(rect.original_setup().camera(camera_index),raw_point);
  const auto d=to_rect_direction(rect,camera_index,raw.direction);
  return calibration::project_camera_point(rect.camera(camera_index),d);
}

core::Image remap_image(const core::Image& source,const core::Image& map_row,const core::Image& map_col){
  if(map_row.pixel_type()!=core::PixelType::Float32||map_col.pixel_type()!=core::PixelType::Float32||map_row.width()!=map_col.width()||map_row.height()!=map_col.height()) throw core::Exception(core::Status::InvalidArgument,"remap_image","maps must be same-size Float32 images");
  std::vector<core::PixelType> types;
  types.reserve(source.channel_count());
  for (std::size_t channel = 0; channel < source.channel_count(); ++channel) types.push_back(source.pixel_type(channel));
  auto out=core::Image::create(map_row.width(),map_row.height(),types);
  for(std::size_t r=0;r<out.height();++r){
    const auto* mr=reinterpret_cast<const float*>(static_cast<const std::byte*>(map_row.data())+r*map_row.stride());
    const auto* mc=reinterpret_cast<const float*>(static_cast<const std::byte*>(map_col.data())+r*map_col.stride());
    for(std::size_t c=0;c<out.width();++c){
      const double sr=mr[c],sc=mc[c];
      for (std::size_t channel = 0; channel < source.channel_count(); ++channel) {
        const auto type = source.pixel_type(channel);
        auto* row = static_cast<std::byte*>(out.mutable_data(channel)) + r * out.stride(channel);
        if(type==core::PixelType::UInt8) reinterpret_cast<std::uint8_t*>(row)[c]=bilerp_scalar<std::uint8_t>(source,sr,sc,channel);
        else if(type==core::PixelType::UInt16) reinterpret_cast<std::uint16_t*>(row)[c]=bilerp_scalar<std::uint16_t>(source,sr,sc,channel);
        else if(type==core::PixelType::Float32) reinterpret_cast<float*>(row)[c]=bilerp_scalar<float>(source,sr,sc,channel);
        else if(type==core::PixelType::Int16) reinterpret_cast<std::int16_t*>(row)[c]=bilerp_scalar<std::int16_t>(source,sr,sc,channel);
        else if(type==core::PixelType::Int32) reinterpret_cast<std::int32_t*>(row)[c]=bilerp_scalar<std::int32_t>(source,sr,sc,channel);
      }
    }
  }
  return out;
}

void map_binocular_images(const core::Image& left,const core::Image& right,const StereoRectification& rect,core::Image* left_rect,core::Image* right_rect){
  if(!left_rect||!right_rect)throw core::Exception(core::Status::InvalidArgument,"map_binocular_images","outputs must not be null");
  auto l=remap_image(left,rect.map_row(0),rect.map_col(0));auto r=remap_image(right,rect.map_row(1),rect.map_col(1));*left_rect=std::move(l);*right_rect=std::move(r);
}

} // namespace mv::reconstruction
