#pragma once

#include <imageio/image_io.hpp>
#include <imageio/registry.hpp>

namespace mv::imageio {

[[nodiscard]] CodecRegistry make_builtin_registry();
[[nodiscard]] ImageIOService& default_image_io_service();

}  // namespace mv::imageio
