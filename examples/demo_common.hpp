// Copyright 2026 MVKit contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <mvkit/cpp/mvkit.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

// 图片读写、叠加显示和命令行辅助；功能算法在各 demo 源文件中展示。
namespace demo {
constexpr int width = 480;
constexpr int height = 320;

inline mv::MvImage load_procedural(const std::string& filename) {
  mv::MvImage image, gray, resized;
  mv::ReadImage(&image, filename.c_str());
  if (image.Channels() == 3) mv::Rgb1ToGray(image, &gray);
  else gray = image;
  mv::ZoomImageSize(gray, &resized, width, height, "bilinear");
  return resized;
}

inline mv::MvImage load_object(const std::string& filename) {
  mv::MvImage image;
  image.ReadImage(filename.c_str());
  const auto gray = image.Channels() == 3 ? image.Rgb1ToGray() : image;
  return gray.ZoomImageSize(width, height, "bilinear");
}

inline void save_procedural(const mv::MvImage& image, const std::string& filename) {
  mv::MvImage full;
  mv::FullDomain(image, &full);
  mv::WriteImage(full, "png", 0, filename.c_str());
}

inline void save_object(const mv::MvImage& image, const std::string& filename) {
  image.FullDomain().WriteImage("png", 0, filename.c_str());
}

inline void save_named(const mv::MvImage& image, const std::string& directory, const char* name) {
  save_procedural(image, directory + "/" + name + ".png");
}

inline mv::MvXLDCont contours_at_pose(const mv::MvXLDCont& contours,
                                     double row, double column, double angle) {
  mv::MvHomMat2D pose;
  pose.VectorAngleToRigid(0, 0, 0, row, column, angle);
  return contours.AffineTransContourXld(pose);
}

inline mv::MvImage add_impulse_noise(const mv::MvImage& image) {
  mv::MvObject copied;
  mv::CopyImage(image, &copied);
  mv::MvImage noisy(copied);
  auto* pixels = static_cast<std::uint8_t*>(noisy.MutableData());
  std::uint32_t state = 20261017;
  for (int row = 0; row < height; ++row) {
    for (int column = 0; column < width; ++column) {
      state = state * 1664525u + 1013904223u;
      if ((state >> 24) < 10)
        pixels[row * noisy.Stride() + column] = state & 1 ? 255 : 0;
    }
  }
  return noisy;
}

inline mv::MvImage region_overlay(const mv::MvImage& gray, const mv::MvRegion& regions,
                                  const char* mode = "margin") {
  return gray.PaintRegion(regions, 40, mode).Compose3(
      gray.PaintRegion(regions, 240, mode), gray.PaintRegion(regions, 150, mode));
}

// 仅栅格化 SDK 返回的轮廓点，不生成或替换检测结果。
inline mv::MvImage contour_overlay(const mv::MvImage& gray, const mv::MvXLDCont& contours) {
  mv::MvImage channels[3];
  std::uint8_t* pixels[3];
  for (std::size_t channel = 0; channel < 3; ++channel) {
    mv::MvObject copied;
    mv::CopyImage(gray, &copied);
    channels[channel] = mv::MvImage(copied);
    pixels[channel] = static_cast<std::uint8_t*>(channels[channel].MutableData());
  }
  for (std::int64_t index = 1; index <= contours.CountObj(); ++index) {
    mv::MvTuple rows, columns;
    contours.SelectObj(index).GetContourXld(&rows, &columns);
    for (std::size_t point = 1; point < rows.Length(); ++point) {
      const double dr = rows.D(point) - rows.D(point - 1);
      const double dc = columns.D(point) - columns.D(point - 1);
      const int steps = std::max(1, int(std::ceil(std::max(std::abs(dr), std::abs(dc)))));
      for (int step = 0; step <= steps; ++step) {
        const int row = int(std::lround(rows.D(point - 1) + dr * step / steps));
        const int column = int(std::lround(columns.D(point - 1) + dc * step / steps));
        if (row < 0 || column < 0 || row >= height || column >= width) continue;
        const int color[] = {40, 240, 150};
        for (std::size_t channel = 0; channel < 3; ++channel)
          pixels[channel][row * channels[channel].Stride() + column] = std::uint8_t(color[channel]);
      }
    }
  }
  return channels[0].Compose3(channels[1], channels[2]);
}

inline void write_pose(const std::string& prefix, const mv::MvTuple& row,
                       const mv::MvTuple& column, const mv::MvTuple& angle,
                       const mv::MvTuple& score) {
  if (row.Length() != 1) throw std::runtime_error("expected one template match");
  std::ofstream output(prefix + "_pose.csv");
  output << "row,column,angle_rad,score\n" << row.D() << ',' << column.D() << ','
         << angle.D() << ',' << score.D() << '\n';
  if (!output) throw std::runtime_error("could not write pose CSV");
  std::cout << prefix << ": row=" << row.D() << " column=" << column.D()
            << " angle=" << angle.D() << " score=" << score.D() << '\n';
}

using Run = void (*)(const std::string& images, const std::string& prefix);

inline int run(int argc, char** argv, const char* name, Run procedural, Run object) {
  if (argc < 3 || argc > 4) {
    std::cerr << "usage: " << argv[0]
              << " <image-directory> <existing-output-directory> [procedural|object|both]\n";
    return 2;
  }
  const std::string mode = argc == 4 ? argv[3] : "both";
  if (mode != "procedural" && mode != "object" && mode != "both") {
    std::cerr << "unknown mode: " << mode << '\n';
    return 2;
  }
  try {
    const std::string images = argv[1], prefix = std::string(argv[2]) + "/" + name;
    if (mode != "object") procedural(images, prefix + "_procedural");
    if (mode != "procedural") object(images, prefix + "_object");
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
}  // namespace demo
