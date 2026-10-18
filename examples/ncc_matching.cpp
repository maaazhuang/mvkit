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
  mv::CropRectangle1(image, &templ, 58, 112, 140, 190);
  mv::MvTuple model, row, column, angle, score;
  mv::CreateNccModel(templ, "auto", -0.35, 0.7, 0.025, "use_polarity", &model);
  mv::FindNccModel(scene, model, -0.35, 0.7, 0.75, 1, 0.5, "true", 0,
                   &row, &column, &angle, &score);
  demo::write_pose(prefix, row, column, angle, score);
  const double half_h = (templ.Height() - 1) / 2., half_w = (templ.Width() - 1) / 2.;
  const auto rows = mv::MvTuple(-half_h).Append(-half_h).Append(half_h).Append(half_h).Append(-half_h);
  const auto columns = mv::MvTuple(-half_w).Append(half_w).Append(half_w).Append(-half_w).Append(-half_w);
  mv::MvXLDCont box, aligned;
  mv::GenContourPolygonXld(&box, rows, columns);
  mv::MvTuple pose;
  mv::VectorAngleToRigid(0, 0, 0, row, column, angle, &pose);
  mv::AffineTransContourXld(box, &aligned, pose);
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
  const auto templ = image.CropRectangle1(58, 112, 140, 190);
  mv::MvNCCModel model;
  model.CreateNccModel(templ, "auto", -0.35, 0.7, 0.025, "use_polarity");
  mv::MvTuple row, column, angle, score;
  model.FindNccModel(scene, -0.35, 0.7, 0.75, 1, 0.5, "true", 0, &row, &column, &angle, &score);
  demo::write_pose(prefix, row, column, angle, score);
  const double half_h = (templ.Height() - 1) / 2., half_w = (templ.Width() - 1) / 2.;
  mv::MvXLDCont box;
  box.GenContourPolygonXld(mv::MvTuple(-half_h).Append(-half_h).Append(half_h).Append(half_h).Append(-half_h),
                          mv::MvTuple(-half_w).Append(half_w).Append(half_w).Append(-half_w).Append(-half_w));
  mv::MvHomMat2D pose;
  pose.VectorAngleToRigid(0, 0, 0, row.D(), column.D(), angle.D());
  const auto aligned = box.AffineTransContourXld(pose);
  demo::save_object(templ, prefix + "_template.png");
  demo::save_object(scene, prefix + "_scene.png");
  demo::save_object(demo::contour_overlay(scene, aligned), prefix + "_result.png");
}
}  // namespace

int main(int argc, char** argv) {
  return demo::run(argc, argv, "ncc_matching", procedural, object);
}
