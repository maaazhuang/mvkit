// Copyright 2026 MVKit contributors
// SPDX-License-Identifier: Apache-2.0
#include "demo_common.hpp"

namespace {
void procedural(const std::string& images, const std::string& prefix) {
  const auto image = demo::load_procedural(images + "/washers.png");
  const auto noisy = demo::add_impulse_noise(image);
  mv::MvImage filtered;
  mv::MedianImage(noisy, &filtered, "circle", 1, "mirrored");
  demo::save_procedural(image, prefix + "_input.png");
  demo::save_procedural(noisy, prefix + "_noisy.png");
  demo::save_procedural(filtered, prefix + "_filtered.png");
  std::cout << prefix << ": median filtering completed\n";
}

void object(const std::string& images, const std::string& prefix) {
  const auto image = demo::load_object(images + "/washers.png");
  const auto noisy = demo::add_impulse_noise(image);
  const auto filtered = noisy.MedianImage("circle", 1, "mirrored");
  demo::save_object(image, prefix + "_input.png");
  demo::save_object(noisy, prefix + "_noisy.png");
  demo::save_object(filtered, prefix + "_filtered.png");
  std::cout << prefix << ": median filtering completed\n";
}
}  // namespace

int main(int argc, char** argv) {
  return demo::run(argc, argv, "image_processing", procedural, object);
}
