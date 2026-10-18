#pragma once
#include "tuple_input.hpp"
#include "iconic_handle.hpp"
#include "xld_cache.hpp"
#include <xld/operators.hpp>
#include <cmath>
#include <limits>
namespace mv::c_api::xld_fit {
inline core::XldContours contours(::MvObject object,const char* op){require_iconic(object,op,"Contours");object->value.require_kind(core::IconicKind::XldContour,op);return core::XldContours{object->value};}
inline std::string_view text(const TupleInput& value){if(value.size()!=1)value.fail(core::ErrorCode::InvalidCardinality,-1,"exactly one string required");return value.string(0);}
inline int rounded(const TupleInput& value){const auto n=std::round(value.scalar_number());if(n<std::numeric_limits<int>::min()||n>std::numeric_limits<int>::max())value.fail(core::ErrorCode::InvalidValue,0,"rounded control outside implementation range");return static_cast<int>(n);}
inline int points(const TupleInput& value,int minimum){const int n=rounded(value);if(n!=-1&&n<minimum)value.fail(core::ErrorCode::InvalidValue,0,"-1 or sufficient sample points required");return n;}
inline int nonnegative(const TupleInput& value){const int n=rounded(value);if(n<0)value.fail(core::ErrorCode::InvalidValue,0,"nonnegative rounded control required");return n;}
inline xld::LineFitOptions line(::MvTuple Algorithm,::MvTuple MaxNumPoints,::MvTuple ClippingEndPoints,::MvTuple Iterations,::MvTuple ClippingFactor,const char* op){
 const TupleInput a(Algorithm,op,"Algorithm"),p(MaxNumPoints,op,"MaxNumPoints"),c(ClippingEndPoints,op,"ClippingEndPoints"),i(Iterations,op,"Iterations"),f(ClippingFactor,op,"ClippingFactor");xld::LineFitOptions o;auto name=text(a);
 if(name=="regression")o.algorithm=xld::LineFitAlgorithm::Regression;else if(name=="huber")o.algorithm=xld::LineFitAlgorithm::Huber;else if(name=="tukey")o.algorithm=xld::LineFitAlgorithm::Tukey;else if(name=="drop")o.algorithm=xld::LineFitAlgorithm::Drop;else if(name=="gauss")o.algorithm=xld::LineFitAlgorithm::Gaussian;else a.fail(core::ErrorCode::UnsupportedValue,0,"unsupported line algorithm");
 o.max_num_points=points(p,2);o.clipping_end_points=nonnegative(c);o.iterations=nonnegative(i);o.clipping_factor=f.scalar_number();return o;
}
inline xld::CircleFitOptions circle(::MvTuple Algorithm,::MvTuple MaxNumPoints,::MvTuple MaxClosureDist,::MvTuple ClippingEndPoints,::MvTuple Iterations,::MvTuple ClippingFactor,const char* op){
 const TupleInput a(Algorithm,op,"Algorithm"),p(MaxNumPoints,op,"MaxNumPoints"),d(MaxClosureDist,op,"MaxClosureDist"),c(ClippingEndPoints,op,"ClippingEndPoints"),i(Iterations,op,"Iterations"),f(ClippingFactor,op,"ClippingFactor");xld::CircleFitOptions o;auto name=text(a);
 if(name=="algebraic")o.algorithm=xld::CircleFitAlgorithm::Algebraic;else if(name=="ahuber")o.algorithm=xld::CircleFitAlgorithm::AlgebraicHuber;else if(name=="atukey")o.algorithm=xld::CircleFitAlgorithm::AlgebraicTukey;else if(name=="geometric")o.algorithm=xld::CircleFitAlgorithm::Geometric;else if(name=="geohuber")o.algorithm=xld::CircleFitAlgorithm::GeometricHuber;else if(name=="geotukey")o.algorithm=xld::CircleFitAlgorithm::GeometricTukey;else a.fail(core::ErrorCode::UnsupportedValue,0,"unsupported circle algorithm");
 o.max_num_points=points(p,3);o.max_closure_dist=d.scalar_number();o.clipping_end_points=nonnegative(c);o.iterations=nonnegative(i);o.clipping_factor=f.scalar_number();o.halcon_controls=true;
 if(name=="geometric"||name=="geohuber"||name=="geotukey")o.iterations=50;
 if(name=="algebraic"||name=="geometric"||o.iterations==0)o.clipping_factor=1;
 return o;
}
inline xld::EllipseFitOptions ellipse(::MvTuple Algorithm,::MvTuple MaxNumPoints,::MvTuple MaxClosureDist,::MvTuple ClippingEndPoints,::MvTuple VossTabSize,::MvTuple Iterations,::MvTuple ClippingFactor,const char* op){
 const TupleInput a(Algorithm,op,"Algorithm"),p(MaxNumPoints,op,"MaxNumPoints"),d(MaxClosureDist,op,"MaxClosureDist"),c(ClippingEndPoints,op,"ClippingEndPoints"),v(VossTabSize,op,"VossTabSize"),i(Iterations,op,"Iterations"),f(ClippingFactor,op,"ClippingFactor");xld::EllipseFitOptions o;auto name=text(a);
 if(name=="fitzgibbon")o.algorithm=xld::EllipseFitAlgorithm::Algebraic;else if(name=="fhuber")o.algorithm=xld::EllipseFitAlgorithm::AlgebraicHuber;else if(name=="ftukey")o.algorithm=xld::EllipseFitAlgorithm::AlgebraicTukey;else if(name=="geometric")o.algorithm=xld::EllipseFitAlgorithm::Geometric;else if(name=="geohuber")o.algorithm=xld::EllipseFitAlgorithm::Huber;else if(name=="geotukey")o.algorithm=xld::EllipseFitAlgorithm::Tukey;else a.fail(core::ErrorCode::UnsupportedValue,0,"voss and focal-point ellipse algorithms are not implemented");
 o.max_num_points=points(p,5);o.max_closure_dist=d.scalar_number();o.clipping_end_points=nonnegative(c);(void)v.scalar_number();o.iterations=nonnegative(i);o.clipping_factor=f.scalar_number();o.min_arc_angle=0;o.halcon_controls=true;
 if(name=="geohuber"||name=="geotukey")o.iterations=50;
 if(name=="fitzgibbon"||name=="geometric"||o.iterations==0)o.clipping_factor=1;
 if(name=="fitzgibbon")o.iterations=0;
 return o;
}
template<class F>auto algorithm(const char* op,F&& f){try{return f();}catch(const core::Exception& e){throw core::Exception(e.status(),e.error_code(),op,e.parameter(),e.parameter_index(),e.message());}}
}
