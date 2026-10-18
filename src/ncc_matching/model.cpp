#include "model.hpp"
#include <core/exception.hpp>
#include <imageproc/gray_pyramid.hpp>
#include <imageproc/pixel_dispatch.hpp>
#include <runtime/execution/execution_context.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>

namespace mv::ncc_matching {
namespace {
constexpr double tau=2*std::numbers::pi;
constexpr std::size_t max_pixels=1048576,max_angles=4096,max_samples=16000000;
[[noreturn]] void bad(const char* operation,const char* parameter,const char* text,
                     core::Status status=core::Status::InvalidArgument) {
  throw core::Exception(status,core::default_error_code(status),operation,parameter,-1,text);
}
// 验证只消费数量及非退化标志；保留均值与能量的原累加顺序，不物化未使用 Patch。
struct ZeroValidation { std::size_t count;bool nondegenerate; };
ZeroValidation validate_zero(const Level& level) {
  const auto pixels=level.width*level.height;
  std::size_t n=0;
  double mean=0;
  for(std::size_t i=0;i<pixels;++i)if(level.mask[i])
    mean+=(level.gray[i]-mean)/double(++n);
  if(n<4)return {0,false};
  double energy=0;
  for(std::size_t i=0;i<pixels;++i)if(level.mask[i]) {
    const auto value=level.gray[i]-mean;energy+=value*value;
  }
  // 调用方只判断是否退化。有限正能量必有非零归一化项，不必再物化并归约其平方和。
  if(!(energy>0) || !std::isfinite(energy))return {0,false};
  return {n,true};
}

struct Normalization {double mean,energy;};
bool normalize(Patch& patch,const Normalization* prepared=nullptr) {
  if(patch.samples.size()<4)return false;
  double mean=prepared?prepared->mean:0;
  if(!prepared)for(std::size_t i=0;i<patch.samples.size();++i)mean+=(patch.samples[i].weight-mean)/double(i+1);
  double energy=prepared?prepared->energy:0;
  if(prepared)for(auto& sample:patch.samples)sample.weight-=mean;
  else for(auto& sample:patch.samples) {sample.weight-=mean;energy+=sample.weight*sample.weight;}
  if(!(energy>0) || !std::isfinite(energy))return false;
  const auto scale=1/std::sqrt(energy);
  patch.weights.reserve(patch.samples.size());
  patch.row_min=patch.row_max=patch.samples.front().row;
  patch.column_min=patch.column_max=patch.samples.front().column;
  // 保留均值、能量与归一化表达式的累加次序；输出权重和边界共用最后一次遍历。
  for(auto& p:patch.samples) {
    p.weight*=scale;
    patch.weights.push_back(p.weight);
    patch.weight_sum+=p.weight;
    patch.row_min=std::min(patch.row_min,p.row);patch.row_max=std::max(patch.row_max,p.row);
    patch.column_min=std::min(patch.column_min,p.column);patch.column_max=std::max(patch.column_max,p.column);
    if(!patch.runs.empty() && patch.runs.back().row==p.row && patch.runs.back().end+1==p.column)patch.runs.back().end=p.column;
    else patch.runs.push_back({p.row,p.column,p.column});
    if(!patch.weight_runs.empty() && patch.weight_runs.back().row==p.row && patch.weight_runs.back().end+1==p.column && patch.weight_runs.back().weight==p.weight)patch.weight_runs.back().end=p.column;
    else patch.weight_runs.push_back({p.row,p.column,p.column,p.weight});
  }
  patch.bounds_prepared=true;
  return true;
}
Patch rotate(const Level& level,double angle,std::pmr::memory_resource* memory=runtime::persist_resource(),
             const Normalization* prepared=nullptr) {
  Patch patch{memory};patch.angle=angle;
  // 掩码为原/旋转栅格提供有界数据相关容量提示；稀疏域不预留完整包围盒。
  patch.samples.reserve(std::count(level.mask.begin(),level.mask.end(),std::uint8_t{1}));
  if(std::abs(std::remainder(angle,tau))<1e-12) {
    patch.origin_row=level.origin_row;patch.origin_column=level.origin_column;
    for(std::size_t r=0;r<level.height;++r)for(std::size_t c=0;c<level.width;++c)
      if(level.mask[r*level.width+c])patch.samples.push_back({int(r),int(c),level.gray[r*level.width+c]});
    if(!normalize(patch,prepared))patch.samples.clear();return patch;
  }
  const auto ca=std::cos(angle),sa=std::sin(angle);
  double rmin=0,rmax=0,cmin=0,cmax=0;
  for(double r : {0.,double(level.height-1)})for(double c : {0.,double(level.width-1)}) {
    const auto dr=r-level.origin_row,dc=c-level.origin_column;
    const auto rr=ca*dr-sa*dc,cc=sa*dr+ca*dc;
    rmin=std::min(rmin,rr);rmax=std::max(rmax,rr);cmin=std::min(cmin,cc);cmax=std::max(cmax,cc);
  }
  // 栅格坐标相对原定义域质心；搜索结果就是该质心，非规则或偏心域也如此。
  const auto rw=std::ceil(rmax)-std::floor(rmin)+1,cw=std::ceil(cmax)-std::floor(cmin)+1;
  if(rw>4*max_pixels/cw)bad("ncc_model_create","domain","rotated bounding box exceeds budget",core::Status::LimitExceeded);
  for(int r=int(std::floor(rmin));r<=int(std::ceil(rmax));++r)
    for(int c=int(std::floor(cmin));c<=int(std::ceil(cmax));++c) {
      const double rr=ca*r+sa*c+level.origin_row,cc=-sa*r+ca*c+level.origin_column;
      if(rr<0 || cc<0 || rr>level.height-1 || cc>level.width-1)continue;
      const auto r0=std::size_t(rr),c0=std::size_t(cc),r1=std::min(r0+1,level.height-1),c1=std::min(c0+1,level.width-1);
      const auto a=r0*level.width+c0,b=r0*level.width+c1,d=r1*level.width+c0,e=r1*level.width+c1;
      if(!level.mask[a] || !level.mask[b] || !level.mask[d] || !level.mask[e])continue;
      const auto fr=rr-r0,fc=cc-c0;
      patch.samples.push_back({r,c,(1-fr)*((1-fc)*level.gray[a]+fc*level.gray[b])+fr*((1-fc)*level.gray[d]+fc*level.gray[e])});
    }
  if(!normalize(patch))patch.samples.clear();return patch;
}
}

Patch make_patch(const Level& level,double angle,std::pmr::memory_resource* memory) {
  return rotate(level,angle,memory);
}

double normalized_correlation(std::span<const double> a,std::span<const double> b,bool absolute) {
  if(a.size()!=b.size() || a.size()<2)bad("ncc_correlation","samples","equal sample arrays with at least two values required");
  // 先用增量均值中心化；常量输入不匹配，绝对 DC/尺度不能通过相消制造分母。
  double ma=0,mb=0;
  for(std::size_t i=0;i<a.size();++i) {
    if(!std::isfinite(a[i]) || !std::isfinite(b[i]))bad("ncc_correlation","samples","finite values required");
    ma+=(a[i]-ma)/double(i+1);mb+=(b[i]-mb)/double(i+1);
  }
  double aa=0,bb=0,ab=0;
  for(std::size_t i=0;i<a.size();++i) {const auto x=a[i]-ma,y=b[i]-mb;aa+=x*x;bb+=y*y;ab+=x*y;}
  if(!(aa>0 && bb>0))return -1;
  const auto score=std::clamp(ab/std::sqrt(aa)/std::sqrt(bb),-1.,1.);
  return absolute?std::abs(score):score;
}

Model create(const core::Image& image,const CreateOptions& options) {
  constexpr auto op="ncc_model_create";
  if(image.empty() || image.channel_count()!=1)bad(op,"image","one nonempty single-channel image required");
  if(!std::isfinite(options.angle_start) || !std::isfinite(options.angle_extent) ||
     !std::isfinite(options.angle_step) || options.angle_extent<0 || options.angle_extent>tau ||
     !std::isfinite(options.angle_start+options.angle_extent) || options.angle_step<0 || options.levels<0 || options.levels>16)bad(op,"options","invalid levels or angle range");
  const auto domain=image.domain();const auto box=domain.bounding_box();
  if(!box || domain.area()<4)bad(op,"domain","template domain requires at least four pixels");
  if(box->row_min<0 || box->col_min<0 || box->row_max>=std::int64_t(image.height()) || box->col_max>=std::int64_t(image.width()))bad(op,"domain","template domain must be inside the image");
  runtime::ExecutionContext context;
  Model model;model.options=options;Level first;
  first.width=std::size_t(box->col_max-box->col_min+1);first.height=std::size_t(box->row_max-box->row_min+1);
  if(first.height>max_pixels/first.width)bad(op,"domain","template bounding box exceeds one million pixels",core::Status::LimitExceeded);
  first.gray.resize(first.width*first.height);first.mask.resize(first.gray.size());
  long double sum_row=0,sum_column=0;
  std::int64_t integer_sum=0;
  bool integer_samples=false;
  imageproc::dispatch_scalar(image.pixel_type(),[&](auto tag) {
    using T=typename decltype(tag)::type;
    integer_samples=std::is_integral_v<T>;
    const auto* base=static_cast<const std::byte*>(image.data());
    const auto stride=image.stride();
    for(const auto& run:domain.runs()) {
      const auto count=std::int64_t(run.col_end)-run.col_begin+1;
      // 有效包围盒至多百万像素，坐标为非负 int32，整数矩的乘积在 Double 精确范围内。
      sum_row+=static_cast<long double>(run.row)*count;
      sum_column+=(static_cast<long double>(run.col_begin)+run.col_end)*count/2;
      const auto* row=base+run.row*stride;
      const auto row_offset=(run.row-box->row_min)*first.width;
      for(auto c=std::int64_t(run.col_begin);c<=run.col_end;++c) {
        T value;std::memcpy(&value,row+c*sizeof(T),sizeof(T));
        if(!std::isfinite(double(value)))bad(op,"image","template samples must be finite");
        const auto index=row_offset+(c-box->col_min);
        first.gray[index]=double(value);first.mask[index]=1;
        if constexpr(std::is_integral_v<T>)integer_sum+=value;
      }
    }
  });
  model.origin_row=double(sum_row/domain.area());model.origin_column=double(sum_column/domain.area());
  // 大绝对坐标舍入前计算相对质心；否则偏心模型即使每个保存 Double 精确往返，也可能不通过自身持久化检查。
  first.origin_row=double((sum_row-static_cast<long double>(box->row_min)*domain.area())/domain.area());
  first.origin_column=double((sum_column-static_cast<long double>(box->col_min)*domain.area())/domain.area());
  // 金字塔滤波及评分累加前移除共享 DC。
  double mean=0;std::size_t n=0;
  // 整数样本总和用 int64 精确累加（上界 2^31*10^6），均值只做一次 Double 除法。
  // 浮点模板保留增量均值以处理较大的 DC 与动态范围。
  if(integer_samples)mean=double(integer_sum)/double(domain.area());
  else for(std::size_t i=0;i<first.gray.size();++i)if(first.mask[i])mean+=(first.gray[i]-mean)/double(++n);
  for(std::size_t i=0;i<first.gray.size();++i)if(first.mask[i])first.gray[i]-=mean;
  // 首层非退化检查与 rebuild 的原始样本准备共用均值和能量，避免重复遍历及逐像素除法。
  return rebuild(std::move(first),options,model.origin_row,model.origin_column);
}

Model rebuild(Level first,const CreateOptions& options,double origin_row,double origin_column) {
  constexpr auto op="ncc_model_create";
  runtime::ExecutionContext context;
  Model model;model.options=options;model.origin_row=origin_row;model.origin_column=origin_column;
  model.sample_count=std::count(first.mask.begin(),first.mask.end(),std::uint8_t{1});
  model.levels.push_back(std::move(first));
  Normalization first_normalization{};
  // 连续拟合始终使用未旋转的原掩码样本；按旧逐查询准备的同一均值、能量及除法序只准备一次，
  // 查询仅共享不可变模型数据。
  {
    const auto& level=model.levels.front();auto& patch=model.original;
    patch.origin_row=level.origin_row;patch.origin_column=level.origin_column;
    patch.samples.reserve(model.sample_count);patch.weights.reserve(model.sample_count);
    patch.relative_rows.reserve(model.sample_count);patch.relative_columns.reserve(model.sample_count);
    double mean=0;std::size_t count=0;
    for(std::size_t i=0;i<level.gray.size();++i)if(level.mask[i])mean+=(level.gray[i]-mean)/double(++count);
    double energy=0;
    for(std::size_t r=0;r<level.height;++r)for(std::size_t c=0;c<level.width;++c)if(level.mask[r*level.width+c]) {
      const auto value=level.gray[r*level.width+c]-mean;energy+=value*value;patch.samples.push_back({int(r),int(c),value});
    }
    if(count<4 || !(energy>0) || !std::isfinite(energy))
      bad(op,"image","constant template cannot define an NCC model");
    first_normalization={mean,energy};
    const double norm=std::sqrt(energy);
    patch.row_min=patch.row_max=patch.samples.front().row;
    patch.column_min=patch.column_max=patch.samples.front().column;
    for(auto& p:patch.samples){p.weight/=norm;patch.weights.push_back(p.weight);
      patch.row_min=std::min(patch.row_min,p.row);patch.row_max=std::max(patch.row_max,p.row);
      patch.column_min=std::min(patch.column_min,p.column);patch.column_max=std::max(patch.column_max,p.column);
      patch.relative_rows.push_back(p.row-level.origin_row);patch.relative_columns.push_back(p.column-level.origin_column);}
    patch.bounds_prepared=true;
  }
  const auto wanted=options.levels?options.levels:16;
  while(model.levels.size()<std::size_t(wanted)) {
    const auto& previous=model.levels.back();
    if(previous.width<8 || previous.height<8)break;
    Level next;next.width=previous.width/2;next.height=previous.height/2;
    next.origin_row=(previous.origin_row-.5)*.5;next.origin_column=(previous.origin_column-.5)*.5;
    next.gray.resize(next.width*next.height);next.mask.resize(next.gray.size());
    for(std::size_t r=0;r<next.height;++r)for(std::size_t c=0;c<next.width;++c) {
      const auto i=2*r*previous.width+2*c;
      next.mask[r*next.width+c]=previous.mask[i] && previous.mask[i+1] && previous.mask[i+previous.width] && previous.mask[i+previous.width+1];
      next.gray[r*next.width+c]=imageproc::box2_gray(previous.gray.data()+2*r*previous.width,previous.gray.data()+(2*r+1)*previous.width,c,previous.width);
    }
    const auto check=validate_zero(next);
    if(!check.nondegenerate || check.count<8 || (model.levels.size()>=2 && check.count<64))break;
    model.levels.push_back(std::move(next));
  }
  if(options.levels && model.levels.size()!=std::size_t(options.levels))bad(op,"levels","requested pyramid level is degenerate or too small");
  const auto radius=std::hypot(double(model.levels.front().width),double(model.levels.front().height))*.5;
  model.options.angle_step=options.angle_step>0 ? options.angle_step : std::max(.01,std::min(.15,1/std::max(1.,radius)));
  const auto intervals=std::ceil(options.angle_extent/model.options.angle_step);
  if(!std::isfinite(intervals) || intervals>=max_angles)bad(op,"angle_step","angle grid exceeds limit",core::Status::LimitExceeded);
  const auto angle_count=std::size_t(intervals)+1;
  std::size_t sample_count=0;
  for(auto& level:model.levels) {
    level.patches.reserve(angle_count);
    for(std::size_t i=0;i<angle_count;++i) {
      const auto angle=options.angle_start+std::min(options.angle_extent,i*model.options.angle_step);
      // 首层零角度使用完全相同的掩码、逐样本均值与能量；仅复用归约，不改变缩放表达式。
      auto patch=rotate(level,angle,runtime::persist_resource(),
                        &level==&model.levels.front()?&first_normalization:nullptr);
      // rotate 仅在 normalize 已证明有限正能量时保留样本，否则返回空 Patch。
      if(patch.samples.empty())bad(op,"domain","rotation produces a degenerate template");
      if(patch.samples.size()>max_samples-sample_count)bad(op,"angle_extent","model raster budget exceeded",core::Status::LimitExceeded);
      sample_count+=patch.samples.size();level.patches.push_back(std::move(patch));
    }
  }
  return model;
}

Model clone(const Model& source) {
  Model result;result.options=source.options;result.sample_count=source.sample_count;
  result.origin_row=source.origin_row;result.origin_column=source.origin_column;
  const auto copy_patch=[](const Patch& input) {
    Patch output;output.angle=input.angle;output.origin_row=input.origin_row;output.origin_column=input.origin_column;
    output.row_min=input.row_min;output.row_max=input.row_max;
    output.column_min=input.column_min;output.column_max=input.column_max;output.bounds_prepared=input.bounds_prepared;
    output.samples.assign(input.samples.begin(),input.samples.end());
    output.weights.assign(input.weights.begin(),input.weights.end());
    output.relative_rows.assign(input.relative_rows.begin(),input.relative_rows.end());
    output.relative_columns.assign(input.relative_columns.begin(),input.relative_columns.end());
    output.runs.assign(input.runs.begin(),input.runs.end());output.weight_sum=input.weight_sum;
    output.weight_runs.assign(input.weight_runs.begin(),input.weight_runs.end());return output;
  };
  result.original=copy_patch(source.original);result.levels.reserve(source.levels.size());
  for(const auto& input:source.levels) {
    Level output;output.width=input.width;output.height=input.height;
    output.origin_row=input.origin_row;output.origin_column=input.origin_column;
    output.gray.assign(input.gray.begin(),input.gray.end());output.mask.assign(input.mask.begin(),input.mask.end());
    output.patches.reserve(input.patches.size());
    for(const auto& patch:input.patches)output.patches.push_back(copy_patch(patch));
    result.levels.push_back(std::move(output));
  }
  return result;
}

} // namespace mv::ncc_matching
