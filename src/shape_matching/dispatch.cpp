#include "gradient_simd.hpp"
#include "runtime/cpu/cpu_features.hpp"

namespace mv::shape_matching::simd {

const Avx2Kernels& avx2_kernels() noexcept {
  static const Avx2Kernels kernels = [] {
    Avx2Kernels result;
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
    if (mv::runtime::cpu_features().selection().avx2) {
      result.score_i8_group8 = &score_i8_group8_avx2;
      result.sobel_u8_i16_row = &sobel_u8_i16_row_avx2;
      result.unit_sobel_f32_row = &unit_sobel_f32_row_avx2;
      result.i8_group_evidence_mask = &i8_group_evidence_mask_avx2;
      result.score_unit_use_polarity = &score_unit_use_polarity_avx2;
      result.score_i16_nearest_use_polarity = &score_i16_nearest_use_polarity_avx2;
      result.prepare_i16_inverse_magnitude = &prepare_i16_inverse_magnitude_avx2;
      result.prepare_i16_packed_components = &prepare_i16_packed_components_avx2;
      result.score_i16_nearest_cached_use_polarity = &score_i16_nearest_cached_use_polarity_avx2;
      result.score_i16_nearest_packed_use_polarity = &score_i16_nearest_packed_use_polarity_avx2;
      result.rotate_model_points = &rotate_model_points_avx2;
      result.score_unit_pre_rotated_use_polarity = &score_unit_pre_rotated_use_polarity_avx2;
      result.score_i16_pre_rotated_use_polarity = &score_i16_pre_rotated_use_polarity_avx2;
      result.score_i16_pre_rotated_cached_use_polarity = &score_i16_pre_rotated_cached_use_polarity_avx2;
      result.score_i16_pre_rotated_packed_use_polarity = &score_i16_pre_rotated_packed_use_polarity_avx2;
      result.score_i16_bilinear_use_polarity = &score_i16_bilinear_use_polarity_avx2;
      result.score_i16_bilinear_packed_use_polarity = &score_i16_bilinear_packed_use_polarity_avx2;
      result.geometric_i16_bilinear_line = &geometric_i16_bilinear_line_avx2;
      result.geometric_i16_bilinear_packed_line = &geometric_i16_bilinear_packed_line_avx2;
      result.geometric_response_peak = &geometric_response_peak_avx2;
      result.geometric_i16_bilinear_peak17 = &geometric_i16_bilinear_peak17_avx2;
      result.geometric_i16_bilinear_packed_peak17 = &geometric_i16_bilinear_packed_peak17_avx2;
      result.xld_geometric_i16_peaks = &xld_geometric_i16_peaks_avx2;
      result.sobel_f32_i16_row = &sobel_f32_i16_row_avx2;
    }
#endif
    return result;
  }();
  return kernels;
}

bool cpu_has_avx2() noexcept {
  return avx2_kernels().score_i8_group8 != nullptr;
}

const char* selected_kernel_name() noexcept {
  if (cpu_has_avx2()) return "avx2";
  if (mv::runtime::cpu_features().selection().sse2) return "sse2";
  return "scalar";
}

std::size_t kernel_table(KernelTableEntry* out, std::size_t capacity) noexcept {
#if MVKIT_SHAPE_HAS_AVX2_KERNEL
  const char* compiled = "avx2";
#else
  const char* compiled = "none";
#endif
#if defined(_M_X64) || defined(__x86_64__)
  const char* mixed_baseline = "scalar/sse2";
#else
  const char* mixed_baseline = "scalar";
#endif
  // 这些值描述可用快路径，不是逐调用执行遥测；极性、形变、ROI、点数及尾部可能回退。
  const auto& kernels = avx2_kernels();
  const KernelTableEntry table[] = {
      {"score_i8_group8", compiled, kernels.score_i8_group8 ? "avx2" : mixed_baseline, mixed_baseline},
      {"sobel_u8_i16_row", compiled, kernels.sobel_u8_i16_row ? "avx2" : mixed_baseline, mixed_baseline},
      {"unit_sobel_f32_row", compiled, kernels.unit_sobel_f32_row ? "avx2" : mixed_baseline, mixed_baseline},
      {"i8_group_evidence_mask", compiled, kernels.i8_group_evidence_mask ? "avx2" : mixed_baseline, mixed_baseline},
      {"score_unit_use_polarity", compiled, kernels.score_unit_use_polarity ? "avx2" : mixed_baseline, mixed_baseline},
      {"score_i16_nearest_use_polarity", compiled,
       kernels.score_i16_nearest_use_polarity ? "avx2" : mixed_baseline,
       mixed_baseline},
      {"rotate_model_points", compiled, kernels.rotate_model_points ? "avx2" : "scalar", "scalar"},
      {"score_unit_pre_rotated_use_polarity", compiled,
       kernels.score_unit_pre_rotated_use_polarity ? "avx2" : mixed_baseline,
       mixed_baseline},
      {"score_i16_pre_rotated_use_polarity", compiled,
       kernels.score_i16_pre_rotated_use_polarity ? "avx2" : mixed_baseline,
       mixed_baseline},
      {"score_i16_bilinear_use_polarity", compiled,
       kernels.score_i16_bilinear_use_polarity ? "avx2" : mixed_baseline,
       mixed_baseline},
      {"geometric_i16_bilinear_line", compiled, kernels.geometric_i16_bilinear_line ? "avx2" : "scalar", "scalar"},
      {"geometric_response_peak", compiled, kernels.geometric_response_peak ? "avx2" : "scalar", "scalar"},
      {"geometric_i16_bilinear_peak17", compiled, kernels.geometric_i16_bilinear_peak17 ? "avx2" : "scalar", "scalar"},
      {"xld_geometric_i16_peaks", compiled, kernels.xld_geometric_i16_peaks ? "avx2" : "scalar", "scalar"},
      {"sobel_f32_i16_row", compiled, kernels.sobel_f32_i16_row ? "avx2" : "scalar", "scalar"},
  };
  const std::size_t count = sizeof(table) / sizeof(table[0]);
  if (out == nullptr || capacity == 0) return count;
  const std::size_t copied = count < capacity ? count : capacity;
  for (std::size_t index = 0; index < copied; ++index) out[index] = table[index];
  return count;
}

}  // namespace mv::shape_matching::simd
