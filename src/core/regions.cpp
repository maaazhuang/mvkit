#include <core/regions.hpp>

#include <utility>

#include <core/exception.hpp>

namespace mv::core {

Regions::Regions(std::initializer_list<Region> regions) {
  if (regions.size() == 1) tuple_ = IconicTuple{IconicValue::from_region(*regions.begin())};
  else if (regions.size()) tuple_ = IconicTuple{IconicValue::from_regions(std::vector<Region>{regions})};
}

Regions::Regions(std::vector<Region> regions) {
  if (regions.size() == 1) tuple_ = IconicTuple{IconicValue::from_region(std::move(regions.front()))};
  else tuple_ = IconicTuple{IconicValue::from_regions(std::move(regions))};
}

Regions::Regions(IconicTuple tuple) : tuple_(std::move(tuple)) {
  tuple_.require_kind(IconicKind::Region, "Regions::Regions");
}

const Region& Regions::operator[](std::size_t index) const {
  if (index >= size()) {
    throw Exception(Status::OutOfRange, ErrorCode::IndexOutOfRange,
                    "Regions::operator[]", "index", static_cast<std::int64_t>(index),
                    "region index is out of range");
  }
  return tuple_.at(index).as_region();
}

}  // namespace mv::core
