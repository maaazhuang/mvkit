// Copyright 2026 MVKit contributors
// SPDX-License-Identifier: Apache-2.0
#include "demo_common.hpp"

namespace {
void procedural(const std::string& images, const std::string& prefix) {
  const auto image = demo::load_procedural(images + "/washers.png");
  mv::MvRegion thresholded, opened, connected, selected, filled;
  mv::Threshold(image, &thresholded, 105, 255);
  mv::OpeningCircle(thresholded, &opened, 1.5);
  mv::Connection(opened, &connected);
  mv::SelectShape(connected, &selected, "area", "and", 150, demo::width * demo::height);
  mv::FillUp(selected, &filled);
  mv::MvTuple count;
  mv::CountObj(selected, &count);
  if (count.I() != 7) throw std::runtime_error("expected seven components");
  demo::save_procedural(image, prefix + "_input.png");
  demo::save_procedural(demo::region_overlay(image, selected), prefix + "_regions.png");
  demo::save_procedural(demo::region_overlay(image, filled, "fill"), prefix + "_filled.png");
  std::cout << prefix << ": components=" << count.I() << '\n';
}

void object(const std::string& images, const std::string& prefix) {
  const auto image = demo::load_object(images + "/washers.png");
  const auto selected = image.Threshold(105, 255).OpeningCircle(1.5).Connection()
      .SelectShape("area", "and", 150, demo::width * demo::height);
  const auto filled = selected.FillUp();
  if (selected.CountObj() != 7) throw std::runtime_error("expected seven components");
  demo::save_object(image, prefix + "_input.png");
  demo::save_object(demo::region_overlay(image, selected), prefix + "_regions.png");
  demo::save_object(demo::region_overlay(image, filled, "fill"), prefix + "_filled.png");
  std::cout << prefix << ": components=" << selected.CountObj() << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  return demo::run(argc, argv, "region_morphology", procedural, object);
}
