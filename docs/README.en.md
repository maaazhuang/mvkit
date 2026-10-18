# mvkit

[简体中文](../README.md) | English

## Overview

**mvkit** is a C/C++ algorithm library for industrial machine vision. It covers image processing, feature extraction, template matching and localization, camera calibration, stereo vision, and 3D processing. Its implementation is independently developed with reference to **HALCON's operator semantics and data object organization**, using forward development to align functionality and performance. The project currently implements **277** commonly used vision operators with HALCON counterparts.

- **The project is under development; API/ABI compatibility is not guaranteed.** Fork the repository or pin a commit/tag for use in your applications.
- The author's professional work focuses on measurement algorithms. This version does not develop or provide **Measure/Metrology** or other measurement-specific interfaces.
- In most tested scenarios, accuracy is broadly consistent with HALCON, with execution time approximately **1–2 times** that of HALCON.

The **C++ procedural and object-oriented interfaces** are recommended. Procedural calls such as `mv::Threshold(image, &regions, ...)` follow an operator-style parameter order. Object-oriented calls such as `image.Threshold(...)` organize application logic around images, regions, and models. Both styles manage resources automatically through objects and tuples and can be combined as needed. The C interface primarily serves language bindings and applications requiring explicit lifetime management.

Initial version: **0.1.0** · [Release notes (Chinese)](version.md).

## Examples

The image processing, contour, and matching results shown below are computed by mvkit. Matching scenes are derived from the same source image using controlled rotation and brightness changes.

Image processing, Region, XLD, Shape matching, and NCC matching each have procedural and object-oriented C++ demonstrations. See [examples](../examples/README.md) for source code and run commands, and [images](../examples/images/) for the inputs. The remaining project documentation is currently in Chinese.

![Median filtering](assets/image-processing.png)

<p align="center"><strong>Filtering</strong> · Input → Impulse noise → Median filter</p>

![Regions and morphology](assets/region-morphology.png)

<p align="center"><strong>Regions and morphology</strong> · Threshold and opening → Connected components → Hole filling</p>

![Subpixel edges](assets/subpixel-edges.png)

<p align="center"><strong>Edge extraction and selection</strong> · Input → Canny subpixel contours → Circular contour selection</p>

![Shape matching](assets/shape-matching.png)

<p align="center"><strong>Shape localization</strong> · Template → Search image → Matched contours and pose</p>

![NCC matching](assets/ncc-matching.png)

<p align="center"><strong>NCC localization</strong> · Template → Search image → Matched pose</p>

## Architecture

```text
Applications / language bindings
        |
Public C++14 procedural / object-oriented interfaces (recommended)
MvImage / MvRegion / MvTuple manage resources automatically
        |
Public C11 interface: opaque objects, control tuples, status and errors
        |
Private C++20: image / region / XLD / matching / calibration / 3D algorithms
        |
runtime: memory resources, scratch space, CPU detection, parallel execution
```

The production shared libraries are `mvkit` and `mvkitcpp`. Internal algorithm libraries are not exported as SDK interfaces. Public headers do not expose Eigen, image codec libraries, or private runtime types.

## Features

| Area | Current capabilities |
|---|---|
| Core objects | Image, Region, XLD, Tuple, Dict, control handles, and errors; iconic objects support 0/1/N elements |
| Image I/O | BMP, JPEG, PNG, TIFF |
| Image processing | Filtering, fixed and adaptive thresholds, arithmetic, colors and channels, ROI, histograms, geometric transforms |
| Regions and morphology | Connected components, set operations, selection, hole filling, dilation/erosion, opening/closing, watershed |
| XLD and subpixel edges | Canny/Sobel edges, contour generation/transformation/segmentation/merging, line/circle/ellipse fitting |
| Shape matching | Image/XLD model creation, rotation and uniform scaling, multiple instances and models, subpixel localization, model I/O |
| NCC matching | Grayscale correlation, rotation, multiple instances and models, polarity control, model I/O |
| Calibration and geometry | Camera models, camera calibration, homography/PnP, distortion correction, projection, plane mapping, hand-eye calibration |
| Stereo and structured light | Camera setup, rectification, SAD/Census/SGM disparity, depth and point clouds; Gray Code, phase-shift and single-stripe reconstruction |
| 3D processing | Point cloud sampling, normals, neighborhoods, distances, fitting, registration, meshing |

The presence of an interface does not imply support for every HALCON parameter combination. See the [documentation index](README.md) for supported subsets, object rules, and feature details.

## Runtime

**Memory pool:** The built-in pool is enabled by default. Persistent objects and scratch space are managed separately, with free blocks reused by size and alignment. Thread-local and shared shards reduce allocation contention and support cross-thread release. Per-operator arenas first use a 4 KiB inline buffer. Cache budgets apply to idle blocks; blocks exceeding the budget are returned to the system. They do not cap live image or point cloud allocations.

**Multithreading:** The framework primarily adapts the range partitioning and task execution approach of OpenCV's [`parallel_for_`](https://docs.opencv.org/4.x/d7/dff/tutorial_how_to_use_OpenCV_parallel_for_.html). It uses a built-in thread pool, shared task chunks, and reusable workers. Execution is serial by default; internal parallelism can be enabled at build time. Small tasks and nested calls may run serially. Set the thread budget with `mv::SetSystem("thread_num", 4)`, preferably before the first parallel operation.

**CPU dispatch:** At runtime, mvkit detects the deployment CPU's instruction sets and the operating system's register-state support. In automatic mode, an available AVX2 implementation is selected when AVX2 kernels were compiled, the CPU and OS support them, and the input meets the kernel's requirements. Other operations use available SSE2 or scalar paths. The project currently has no NEON implementation because an ARM development environment is unavailable. See [runtime](runtime.md) for configuration and usage constraints.

## Build and Use

Building the library requires a C++20 compiler; downstream C++ applications can use C++14. CMake 3.20+ is required, or CMake 4.2+ for the VS2026 generator. Dependencies are first resolved from installed development libraries; missing dependencies are built from fixed-version upstream archives with pinned SHA-256 hashes.

Windows supports VS2019/2022/2026. The script automatically prefers the newest supported installation:

```powershell
./scripts/setup-windows.ps1
New-Item -ItemType Directory -Path build/demo-output -Force
./build/bin/Release/mvkit_demo_image_processing.exe ./examples/images ./build/demo-output
```

Ubuntu / WSL:

```sh
bash scripts/setup-ubuntu.sh --system
cmake --preset linux
cmake --build --preset linux --parallel
mkdir -p build/demo-output
./build/bin/mvkit_demo_image_processing ./examples/images ./build/demo-output
```

Install the SDK:

```sh
cmake --install build --config Release --prefix out/mvkit
```

Downstream `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_vision_app LANGUAGES CXX)
find_package(MVKit CONFIG REQUIRED)
add_executable(my_vision_app main.cpp)
target_link_libraries(my_vision_app PRIVATE mvkit::CXX)
target_compile_features(my_vision_app PRIVATE cxx_std_14)
```

`main.cpp`:

```cpp
#include <mvkit/cpp/mvkit.hpp>

int main() {
  mv::MvImage image, gray, filtered;
  mv::ReadImage(&image, "examples/images/washers.png");
  mv::Rgb1ToGray(image, &gray);
  mv::MedianImage(gray, &filtered, "circle", 1, "mirrored");
  mv::MvRegion regions;
  mv::Threshold(filtered, &regions, 105, 255);
  mv::WriteImage(filtered, "png", 0, "filtered.png");
}
```

Set `-DCMAKE_PREFIX_PATH=/absolute/path/to/out/mvkit` when configuring downstream projects. The CMake targets automatically pass `/utf-8` to MSVC consumers for the UTF-8 public headers. Windows C++ applications must be able to locate `mvkit.dll`, `mvkitcpp.dll`, and any required codec runtime libraries. See the [build guide](build.md) for full configuration and offline builds.

## Development Environment

The main development machine uses an Intel Core i5-13600K (14 cores / 20 threads), approximately 64 GiB of RAM, Windows 11 x64, and Visual Studio 2026 / MSVC 19.51. Linux builds use WSL2 with Ubuntu 24.04.4 LTS, GCC 13.3, CMake 3.28.3, and Ninja 1.11.1. See [development and validation environments](platform.md) for details.

## Additional Notes

- This repository contains the public source subset. Development tests, benchmarks, and legacy C/C++ examples based on synthetic arrays are not included.
- Models and serialized data use mvkit's own formats, which are not compatible with HALCON binary model formats.
- HALCON is used only in an independent development environment for public-interface reference and normal comparison testing. It is not a dependency, and its images, models, and runtime libraries are not distributed here.
- Original code and synthetic assets are provided under [Apache-2.0](../LICENSE). Third-party components retain their original licenses. See [NOTICE](../NOTICE) and [third-party components and licenses](#third-party-components-and-licenses).

## Third-Party Components and Licenses

MVKit's original code is licensed under the [Apache License 2.0](../LICENSE). This does not replace the original licenses of third-party components, images, models, or other assets. Copyright and license notices in component source files remain valid. Project notices are in [NOTICE](../NOTICE).

The following are the versions bundled with the repository or pinned for source fallback builds. Actual system library versions and transitive dependencies depend on the build and distributed package. Fixed download URLs and SHA-256 hashes are recorded in [Dependencies.cmake](../cmake/Dependencies.cmake).

| Component | Version and use | Original license and source |
|---|---|---|
| Eigen | 3.4.0; optional private numerical backend, headers bundled from CasADi's Eigen tree; production code uses Eigen/Dense | MPL-2.0 and per-file notices; [source record](../3rdparty/eigen/README.MVKit), [license text](../3rdparty/eigen/COPYING.MPL2), [upstream](https://eigen.tuxfamily.org/) |
| libjpeg-turbo | 3.1.4.1; JPEG source fallback | IJG, BSD, and zlib-related licenses; scope is defined by the original LICENSE.md, README.ijg, and per-file notices; [upstream license](https://github.com/libjpeg-turbo/libjpeg-turbo/blob/3.1.4.1/LICENSE.md) |
| libpng | 1.6.58; PNG source fallback | PNG Reference Library License and historical notices; [upstream license](https://github.com/pnggroup/libpng/blob/v1.6.58/LICENSE) |
| LibTIFF | 4.7.2; TIFF source fallback | Upstream permissive license and per-file notices; [upstream license](https://gitlab.com/libtiff/libtiff/-/blob/v4.7.2/LICENSE.md) |
| zlib | 1.3.2; source fallback for transitive codec dependencies | zlib License; [upstream license](https://github.com/madler/zlib/blob/v1.3.2/LICENSE) |

When using libjpeg-turbo, retain the following notice:

> This software is based in part on the work of the Independent JPEG Group.
