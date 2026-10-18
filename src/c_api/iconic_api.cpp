#include <mvkit/c/mv_iconic.h>

#include <atomic>
#include <memory>

#include "error_state.hpp"
#include "iconic_handle.hpp"
#include "tuple_input.hpp"

namespace {

::MvIconicKind to_c_kind(mv::core::IconicKind kind) noexcept {
  switch (kind) {
    case mv::core::IconicKind::Image: return MV_ICONIC_IMAGE;
    case mv::core::IconicKind::Region: return MV_ICONIC_REGION;
    case mv::core::IconicKind::XldContour: return MV_ICONIC_XLD_CONTOUR;
  }
  return MV_ICONIC_IMAGE;
}

}  // namespace

extern "C" ::MvStatus mv_clear_obj(::MvObject objects) {
  return mv::c_api::translate([&] {
    mv::c_api::require_iconic(objects, "mv_clear_obj");
    if (objects->refs.fetch_sub(1, std::memory_order_acq_rel) == 1U) delete objects;
  });
}

extern "C" ::MvStatus mv_gen_empty_obj(::MvObject* objects) {
  return mv::c_api::translate([&] {
    mv::c_api::require_output(objects, "gen_empty_obj", "EmptyObject");
    auto created = mv::c_api::make_iconic({});
    *objects = created.release();
  });
}

extern "C" ::MvStatus mv_count_obj(::MvObject objects, ::MvTuple* count) {
  return mv::c_api::translate([&] {
    mv::c_api::require_iconic(objects, "count_obj", "Objects");
    mv::c_api::require_output(count, "count_obj", "Number");
    auto created = std::make_unique<MvTupleImpl>();
    created->value.append(static_cast<std::int64_t>(objects->value.size()));
    *count = created.release();
  });
}

extern "C" ::MvStatus mv_select_obj(::MvObject objects, ::MvObject* selected,
                                    ::MvTuple index) {
  return mv::c_api::translate([&] {
    mv::c_api::require_iconic(objects, "select_obj", "Objects");
    mv::c_api::require_output(selected, "select_obj", "SelectedObject");
    const mv::c_api::TupleInput positions(index, "select_obj", "Index");
    std::vector<mv::core::IconicValue> values;
    if (!objects->value.empty()) {
      values.reserve(positions.size());
      for (std::size_t i=0; i<positions.size(); ++i) {
        const auto position=positions.integer(i);
        if (position<1 || static_cast<std::uint64_t>(position)>objects->value.size())
          throw mv::core::Exception(mv::core::Status::OutOfRange, mv::core::ErrorCode::IndexOutOfRange,
              "select_obj", "Index", i, "object index is outside the 1-based tuple range");
        values.push_back(objects->value.at(static_cast<std::size_t>(position-1)));
      }
    }
    auto created = mv::c_api::make_iconic(mv::core::IconicTuple(std::move(values)));
    *selected = created.release();
  });
}

extern "C" ::MvStatus mv_concat_obj(::MvObject left, ::MvObject right,
                                    ::MvObject* concatenated) {
  return mv::c_api::translate([&] {
    mv::c_api::require_iconic(left, "concat_obj", "Objects1");
    mv::c_api::require_iconic(right, "concat_obj", "Objects2");
    mv::c_api::require_output(concatenated, "concat_obj", "ObjectsConcat");
    auto created = mv::c_api::make_iconic(left->value.concat(right->value));
    *concatenated = created.release();
  });
}

extern "C" ::MvStatus mv_copy_obj(::MvObject objects, ::MvObject* copied,
                                  ::MvTuple index, ::MvTuple num_obj) {
  return mv::c_api::translate([&] {
    mv::c_api::require_iconic(objects, "copy_obj", "Objects");
    mv::c_api::require_output(copied, "copy_obj", "ObjectsSelected");
    const mv::c_api::TupleInput first(index, "copy_obj", "Index");
    const mv::c_api::TupleInput count(num_obj, "copy_obj", "NumObj");
    auto created = mv::c_api::make_iconic(objects->value.copy(first.scalar_integer(), count.scalar_integer()));
    *copied = created.release();
  });
}

extern "C" ::MvStatus mv_get_obj_class(::MvObject Objects,::MvTuple* Classes) {
  return mv::c_api::translate([&] {
    mv::c_api::require_iconic(Objects,"get_obj_class","Objects");
    mv::c_api::require_output(Classes,"get_obj_class","Classes");
    auto result=std::make_unique<MvTupleImpl>();
    for(std::size_t i=0;i<Objects->value.size();++i) {
      const auto kind=Objects->value.at(i).kind();
      result->value.append(kind==mv::core::IconicKind::Image?"image":kind==mv::core::IconicKind::Region?"region":"xld_cont");
    }
    *Classes=result.release();
  });
}
extern "C" ::MvStatus mv_copy_image(::MvObject Image,::MvObject* DupImage) {
  return mv::c_api::translate([&] {
    mv::c_api::require_iconic(Image,"copy_image","Image");
    mv::c_api::require_output(DupImage,"copy_image","DupImage");
    Image->value.require_kind(mv::core::IconicKind::Image,"copy_image");
    auto result=mv::c_api::make_iconic(Image->value.clone());
    *DupImage=result.release();
  });
}
