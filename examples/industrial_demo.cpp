// Copyright 2026 MVKit contributors
// SPDX-License-Identifier: Apache-2.0
#include "demo_common.hpp"

#include <initializer_list>

namespace {
using demo::width;
using demo::height;

// 将已计算的结果等比排成无文字三联图；仅处理显示布局。
void save_panel(const std::string& output, const char* name,
                std::initializer_list<const char*> inputs) {
  constexpr int margin = 12, gap = 12, panel_width = 1488, panel_height = 344;
  mv::MvImage channels[3];
  std::uint8_t* pixels[3];
  const int background[] = {246, 248, 250}, border[] = {208, 215, 222}, dark[] = {17, 25, 35};
  for (int channel = 0; channel < 3; ++channel) {
    channels[channel].GenImageConst("byte", panel_width, panel_height);
    pixels[channel] = static_cast<std::uint8_t*>(channels[channel].MutableData());
    for (int row = 0; row < panel_height; ++row)
      std::fill_n(pixels[channel] + row * channels[channel].Stride(), panel_width, background[channel]);
  }
  int index = 0;
  for (const auto* input : inputs) {
    mv::MvImage image;
    image.ReadImage((output + "/" + input + ".png").c_str());
    const double scale = std::min(double(width) / image.Width(), double(height) / image.Height());
    const int tile_width = std::min(width, std::max(1, int(std::lround(image.Width() * scale))));
    const int tile_height = std::min(height, std::max(1, int(std::lround(image.Height() * scale))));
    const auto tile = image.ZoomImageSize(tile_width, tile_height, "bilinear");
    const int left = margin + index++ * (width + gap);
    for (int channel = 0; channel < 3; ++channel) {
      for (int row = -1; row <= height; ++row) {
        for (int col = -1; col <= width; ++col) {
          const bool frame = row == -1 || row == height || col == -1 || col == width;
          pixels[channel][(margin + row) * channels[channel].Stride() + left + col] =
              std::uint8_t(frame ? border[channel] : dark[channel]);
        }
      }
      const std::size_t source_channel = tile.Channels() == 1 ? 0 : channel;
      const auto* source = static_cast<const std::uint8_t*>(tile.Data(source_channel));
      for (int row = 0; row < tile_height; ++row) {
        auto* destination = pixels[channel] +
            (margin + (height - tile_height) / 2 + row) * channels[channel].Stride() +
            left + (width - tile_width) / 2;
        std::copy_n(source + row * tile.Stride(source_channel), tile_width, destination);
      }
    }
  }
  demo::save_named(channels[0].Compose3(channels[1], channels[2]), output, name);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: mvkit_industrial_demo <image-directory> <existing-output-directory>\n";
    return 2;
  }
  try {
    const std::string assets = argv[1], output = argv[2];
    auto washers = demo::load_procedural(assets + "/washers.png");
    auto bracket = demo::load_procedural(assets + "/bracket.png");
    demo::save_named(washers, output, "washers_input");
    demo::save_named(bracket, output, "bracket_input");

    const auto noisy = demo::add_impulse_noise(washers);
    demo::save_named(noisy, output, "washers_noisy");
    mv::MvImage filtered;
    mv::MedianImage(noisy, &filtered, "circle", 1, "mirrored");
    demo::save_named(filtered, output, "washers_filtered");
    mv::MvRegion thresholded, opened, connected, regions;
    mv::Threshold(washers, &thresholded, 105, 255);
    mv::OpeningCircle(thresholded, &opened, 1.5);
    mv::Connection(opened, &connected);
    mv::SelectShape(connected, &regions, "area", "and", 150, width * height);
    if (regions.CountObj() != 7) throw std::runtime_error("expected seven separated components");
    mv::MvImage black;
    black.GenImageConst("byte", width, height);
    demo::save_named(black.PaintRegion(regions, 255, "fill"), output, "washers_mask");
    auto labeled = washers.Compose3(washers, washers);
    const int palette[][3] = {{40, 240, 150}, {255, 185, 65}, {75, 185, 255}, {240, 100, 180}};
    for (std::int64_t index = 1; index <= regions.CountObj(); ++index) {
      const auto region = regions.SelectObj(index).DilationCircle(1.0);
      mv::MvImage green, blue;
      const auto red = labeled.Decompose3(&green, &blue);
      const auto& color = palette[(index - 1) % 4];
      labeled = red.PaintRegion(region, color[0], "margin").Compose3(
          green.PaintRegion(region, color[1], "margin"), blue.PaintRegion(region, color[2], "margin"));
    }
    demo::save_named(labeled, output, "washers_regions");
    demo::save_named(demo::region_overlay(washers, regions.FillUp(), "fill"), output, "washers_filled");

    mv::MvXLDCont all_edges, edges;
    mv::EdgesSubPix(bracket, &all_edges, "canny", 1.2, 18, 40);
    mv::SelectContoursXld(all_edges, &edges, "contour_length", 15, 1e9, -0.5, 0.5);
    if (edges.CountObj() == 0) throw std::runtime_error("edge extraction returned no contours");
    demo::save_named(demo::contour_overlay(bracket, edges), output, "bracket_edges");
    mv::MvXLDCont closed_edges, circular_edges;
    mv::SelectContoursXld(edges, &closed_edges, "closed", 0, 1.0, 0, 0);
    mv::SelectShapeXld(closed_edges, &circular_edges, "circularity", "and", 0.9, 1.0);
    if (circular_edges.CountObj() == 0 || circular_edges.CountObj() >= edges.CountObj())
      throw std::runtime_error("circularity selection did not isolate round contours");
    demo::save_named(demo::contour_overlay(bracket, circular_edges), output, "bracket_circular_edges");

    // 搜索场景由同一输入确定性旋转并改变亮度，演示受控变换下的定位。
    mv::MvHomMat2D rotation;
    rotation.VectorAngleToRigid(height / 2., width / 2., 0,
                                height / 2. + 5, width / 2. + 8, 0.24);
    const auto scene = bracket.AffineTransImage(rotation, "bilinear", "false").ScaleImage(0.86, 12);
    const auto templ = bracket.CropRectangle1(48, 100, 245, 390);
    mv::MvTuple shape;
    mv::CreateShapeModel(templ, 3, -0.35, 0.7, 0.025, "auto", "use_polarity", 35, 10, &shape);
    mv::MvTuple row, column, angle, score;
    mv::FindShapeModel(scene, shape, -0.35, 0.7, 0.5, 1, 0.5, "interpolation", 3, 0.8,
                      &row, &column, &angle, &score);
    if (row.Length() != 1) throw std::runtime_error("shape matching did not find one instance");
    mv::MvXLDCont model_contours;
    mv::GetShapeModelContours(&model_contours, shape, 1);
    const auto aligned = demo::contours_at_pose(model_contours, row.D(), column.D(), angle.D());
    demo::save_named(templ, output, "shape_template");
    demo::save_named(scene, output, "shape_scene");
    demo::save_named(demo::contour_overlay(scene, aligned), output, "shape_result");
    const double shape_score = score.D(), shape_angle = angle.D();

    const auto ncc_template = bracket.CropRectangle1(58, 112, 140, 190);
    mv::MvTuple ncc;
    mv::CreateNccModel(ncc_template, "auto", -0.35, 0.7, 0.025, "use_polarity", &ncc);
    mv::FindNccModel(scene, ncc, -0.35, 0.7, 0.75, 1, 0.5, "true", 0,
                    &row, &column, &angle, &score);
    if (row.Length() != 1) throw std::runtime_error("NCC matching did not find one instance");
    mv::MvXLDCont box;
    const double half_h = (ncc_template.Height() - 1) / 2.;
    const double half_w = (ncc_template.Width() - 1) / 2.;
    box.GenContourPolygonXld(mv::MvTuple(-half_h).Append(-half_h).Append(half_h).Append(half_h).Append(-half_h),
                            mv::MvTuple(-half_w).Append(half_w).Append(half_w).Append(-half_w).Append(-half_w));
    demo::save_named(ncc_template, output, "ncc_template");
    demo::save_named(scene, output, "ncc_scene");
    demo::save_named(demo::contour_overlay(scene, demo::contours_at_pose(box, row.D(), column.D(), angle.D())), output, "ncc_result");
    std::ofstream report(output + "/results.json");
    report << "{\n  \"components\": " << regions.CountObj()
           << ",\n  \"edge_contours\": " << edges.CountObj()
           << ",\n  \"circular_contours\": " << circular_edges.CountObj()
           << ",\n  \"shape_score\": " << shape_score
           << ",\n  \"shape_angle_rad\": " << shape_angle
           << ",\n  \"ncc_score\": " << score.D()
           << ",\n  \"ncc_angle_rad\": " << angle.D() << "\n}\n";
    if (!report) throw std::runtime_error("could not write results.json");
    save_panel(output, "image-processing", {"washers_input", "washers_noisy", "washers_filtered"});
    save_panel(output, "region-morphology", {"washers_mask", "washers_regions", "washers_filled"});
    save_panel(output, "subpixel-edges", {"bracket_input", "bracket_edges", "bracket_circular_edges"});
    save_panel(output, "shape-matching", {"shape_template", "shape_scene", "shape_result"});
    save_panel(output, "ncc-matching", {"ncc_template", "ncc_scene", "ncc_result"});
    std::cout << "gallery generated: components=" << regions.CountObj()
              << " contours=" << edges.CountObj() << " circular=" << circular_edges.CountObj()
              << " shape=" << shape_score
              << " ncc=" << score.D() << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
