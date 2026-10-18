// Copyright 2026 MVKit contributors
// SPDX-License-Identifier: Apache-2.0
#include "demo_common.hpp"

namespace {
void procedural(const std::string& images, const std::string& prefix) {
  const auto image = demo::load_procedural(images + "/bracket.png");
  mv::MvTuple rotation;
  mv::VectorAngleToRigid(demo::height / 2., demo::width / 2., 0,
                        demo::height / 2. + 5, demo::width / 2. + 8, 0.24, &rotation);
  mv::MvImage rotated, scene, templ;
  mv::AffineTransImage(image, &rotated, rotation, "bilinear", "false");
  mv::ScaleImage(rotated, &scene, 0.86, 12);
  mv::CropRectangle1(image, &templ, 48, 100, 245, 390);
  mv::MvTuple model, row, column, angle, score;
  mv::CreateShapeModel(templ, 3, -0.35, 0.7, 0.025, "auto", "use_polarity", 35, 10, &model);
  mv::FindShapeModel(scene, model, -0.35, 0.7, 0.5, 1, 0.5, "interpolation", 3, 0.8,
                     &row, &column, &angle, &score);
  demo::write_pose(prefix, row, column, angle, score);
  mv::MvXLDCont contours, aligned;
  mv::GetShapeModelContours(&contours, model, 1);
  mv::MvTuple pose;
  mv::VectorAngleToRigid(0, 0, 0, row, column, angle, &pose);
  mv::AffineTransContourXld(contours, &aligned, pose);
  demo::save_procedural(templ, prefix + "_template.png");
  demo::save_procedural(scene, prefix + "_scene.png");
  demo::save_procedural(demo::contour_overlay(scene, aligned), prefix + "_result.png");
}

void object(const std::string& images, const std::string& prefix) {
  const auto image = demo::load_object(images + "/bracket.png");
  mv::MvHomMat2D rotation;
  rotation.VectorAngleToRigid(demo::height / 2., demo::width / 2., 0,
                             demo::height / 2. + 5, demo::width / 2. + 8, 0.24);
  const auto scene = image.AffineTransImage(rotation, "bilinear", "false").ScaleImage(0.86, 12);
  const auto templ = image.CropRectangle1(48, 100, 245, 390);
  mv::MvShapeModel model;
  model.CreateShapeModel(templ, 3, -0.35, 0.7, 0.025, "auto", "use_polarity", 35, 10);
  mv::MvTuple row, column, angle, score;
  model.FindShapeModel(scene, -0.35, 0.7, 0.5, 1, 0.5, "interpolation", 3, 0.8,
                       &row, &column, &angle, &score);
  demo::write_pose(prefix, row, column, angle, score);
  mv::MvHomMat2D pose;
  pose.VectorAngleToRigid(0, 0, 0, row.D(), column.D(), angle.D());
  const auto aligned = model.GetShapeModelContours(1).AffineTransContourXld(pose);
  demo::save_object(templ, prefix + "_template.png");
  demo::save_object(scene, prefix + "_scene.png");
  demo::save_object(demo::contour_overlay(scene, aligned), prefix + "_result.png");
}
}  // namespace

int main(int argc, char** argv) {
  return demo::run(argc, argv, "shape_matching", procedural, object);
}
