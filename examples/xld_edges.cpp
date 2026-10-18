// Copyright 2026 MVKit contributors
// SPDX-License-Identifier: Apache-2.0
#include "demo_common.hpp"

namespace {
void procedural(const std::string& images, const std::string& prefix) {
  const auto image = demo::load_procedural(images + "/bracket.png");
  mv::MvXLDCont all, edges, closed, circular;
  mv::EdgesSubPix(image, &all, "canny", 1.2, 18, 40);
  mv::SelectContoursXld(all, &edges, "contour_length", 15, 1e9, -0.5, 0.5);
  mv::SelectContoursXld(edges, &closed, "closed", 0, 1.0, 0, 0);
  mv::SelectShapeXld(closed, &circular, "circularity", "and", 0.9, 1.0);
  mv::MvTuple total, count;
  mv::CountObj(edges, &total);
  mv::CountObj(circular, &count);
  if (count.I() == 0 || count.I() >= total.I())
    throw std::runtime_error("expected a subset of circular contours");
  demo::save_procedural(image, prefix + "_input.png");
  demo::save_procedural(demo::contour_overlay(image, edges), prefix + "_edges.png");
  demo::save_procedural(demo::contour_overlay(image, circular), prefix + "_circular.png");
  std::cout << prefix << ": contours=" << total.I() << " circular=" << count.I() << '\n';
}

void object(const std::string& images, const std::string& prefix) {
  const auto image = demo::load_object(images + "/bracket.png");
  const auto edges = image.EdgesSubPix("canny", 1.2, 18, 40)
      .SelectContoursXld("contour_length", 15, 1e9, -0.5, 0.5);
  const auto circular = edges.SelectContoursXld("closed", 0, 1.0, 0, 0)
      .SelectShapeXld("circularity", "and", 0.9, 1.0);
  if (circular.CountObj() == 0 || circular.CountObj() >= edges.CountObj())
    throw std::runtime_error("expected a subset of circular contours");
  demo::save_object(image, prefix + "_input.png");
  demo::save_object(demo::contour_overlay(image, edges), prefix + "_edges.png");
  demo::save_object(demo::contour_overlay(image, circular), prefix + "_circular.png");
  std::cout << prefix << ": contours=" << edges.CountObj()
            << " circular=" << circular.CountObj() << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  return demo::run(argc, argv, "xld_edges", procedural, object);
}
