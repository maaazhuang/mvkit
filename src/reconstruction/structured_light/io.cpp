#include <reconstruction/structured_light/io.hpp>

#include <bit>
#include <cstdint>
#include <fstream>
#include <string>
#include <string_view>

#include <calibration/camera_parameters.hpp>
#include <calibration/geometry3d.hpp>
#include <core/exception.hpp>
#include <reconstruction/structured_light/calibration.hpp>

namespace mv::reconstruction {
namespace {

constexpr std::string_view kMagic{"MVKSLM\r\n", 8};
constexpr std::uint32_t kVersion = 2;
constexpr std::uint32_t kMaximumTupleSize = 32;
constexpr std::uint32_t kMaximumStringSize = 128;

[[noreturn]] void io_error(std::string_view message) {
  throw core::Exception(core::Status::IoError, "read_structured_light_model", message);
}

void write_bytes(std::ostream& out, const void* data, std::size_t size) {
  out.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
  if (!out) {
    throw core::Exception(core::Status::IoError, "write_structured_light_model",
                          "failed while writing model file");
  }
}

void write_u8(std::ostream& out, std::uint8_t value) { write_bytes(out, &value, sizeof(value)); }

void write_u32(std::ostream& out, std::uint32_t value) {
  std::uint8_t bytes[4];
  for (unsigned i = 0; i < 4; ++i) bytes[i] = static_cast<std::uint8_t>(value >> (8U * i));
  write_bytes(out, bytes, sizeof(bytes));
}

void write_u64(std::ostream& out, std::uint64_t value) {
  std::uint8_t bytes[8];
  for (unsigned i = 0; i < 8; ++i) bytes[i] = static_cast<std::uint8_t>(value >> (8U * i));
  write_bytes(out, bytes, sizeof(bytes));
}

void write_i64(std::ostream& out, std::int64_t value) {
  write_u64(out, std::bit_cast<std::uint64_t>(value));
}

void write_real64(std::ostream& out, double value) {
  write_u64(out, std::bit_cast<std::uint64_t>(value));
}

void write_string(std::ostream& out, std::string_view value) {
  if (value.size() > kMaximumStringSize) {
    throw core::Exception(core::Status::LimitExceeded, "write_structured_light_model",
                          "model string exceeds format limit");
  }
  write_u32(out, static_cast<std::uint32_t>(value.size()));
  write_bytes(out, value.data(), value.size());
}

void write_tuple(std::ostream& out, const core::Tuple& tuple) {
  if (tuple.size() > kMaximumTupleSize) {
    throw core::Exception(core::Status::LimitExceeded, "write_structured_light_model",
                          "model tuple exceeds format limit");
  }
  write_u32(out, static_cast<std::uint32_t>(tuple.size()));
  for (std::size_t index = 0; index < tuple.size(); ++index) {
    const auto value = tuple.at(index);
    switch (value.kind()) {
      case core::ValueKind::Int64:
        write_u8(out, 2);
        write_i64(out, value.as_int64());
        break;
      case core::ValueKind::Real64:
        write_u8(out, 3);
        write_real64(out, value.as_real64());
        break;
      case core::ValueKind::String:
        write_u8(out, 4);
        write_string(out, value.as_string().view());
        break;
      default:
        throw core::Exception(core::Status::Unsupported, "write_structured_light_model",
                              "model tuple contains an unsupported value type");
    }
  }
}

void read_bytes(std::istream& in, void* data, std::size_t size) {
  in.read(static_cast<char*>(data), static_cast<std::streamsize>(size));
  if (!in) io_error("truncated or unreadable model file");
}

std::uint8_t read_u8(std::istream& in) {
  std::uint8_t value{};
  read_bytes(in, &value, sizeof(value));
  return value;
}

std::uint32_t read_u32(std::istream& in) {
  std::uint8_t bytes[4];
  read_bytes(in, bytes, sizeof(bytes));
  std::uint32_t value = 0;
  for (unsigned i = 0; i < 4; ++i) value |= static_cast<std::uint32_t>(bytes[i]) << (8U * i);
  return value;
}

std::uint64_t read_u64(std::istream& in) {
  std::uint8_t bytes[8];
  read_bytes(in, bytes, sizeof(bytes));
  std::uint64_t value = 0;
  for (unsigned i = 0; i < 8; ++i) value |= static_cast<std::uint64_t>(bytes[i]) << (8U * i);
  return value;
}

std::int64_t read_i64(std::istream& in) {
  return std::bit_cast<std::int64_t>(read_u64(in));
}

double read_real64(std::istream& in) { return std::bit_cast<double>(read_u64(in)); }

std::string read_string(std::istream& in) {
  const auto size = read_u32(in);
  if (size > kMaximumStringSize) io_error("model string exceeds format limit");
  std::string value(size, '\0');
  read_bytes(in, value.data(), value.size());
  return value;
}

core::Tuple read_tuple(std::istream& in) {
  const auto count = read_u32(in);
  if (count > kMaximumTupleSize) io_error("model tuple exceeds format limit");
  core::Tuple tuple;
  for (std::uint32_t index = 0; index < count; ++index) {
    switch (read_u8(in)) {
      case 1: {
        const auto value = read_u8(in);
        if (value > 1) io_error("invalid boolean in model file");
        // v1 文件可能含已撤出的 Bool kind，读取时归一化为 HALCON 风格整数表示。
        tuple.append(static_cast<std::int64_t>(value));
        break;
      }
      case 2: tuple.append(read_i64(in)); break;
      case 3: tuple.append(read_real64(in)); break;
      case 4: tuple.append(core::String{read_string(in)}); break;
      default: io_error("unknown tuple value type in model file");
    }
  }
  return tuple;
}

const char* orientation_name(PatternOrientation value) {
  switch (value) {
    case PatternOrientation::Vertical: return "vertical";
    case PatternOrientation::Horizontal: return "horizontal";
    case PatternOrientation::Both: return "both";
  }
  return "both";
}

const char* pattern_type_name(StructuredLightPatternType value) {
  switch (value) {
    case StructuredLightPatternType::GrayCode: return "gray_code";
    case StructuredLightPatternType::GrayCodeAndPhaseShift: return "gray_code_and_phase_shift";
    case StructuredLightPatternType::SingleStripe: return "single_stripe";
  }
  return "gray_code_and_phase_shift";
}

const char* normalization_name(StructuredLightNormalization value) {
  return value == StructuredLightNormalization::Global ? "global" : "inverted_pattern";
}

void set_value(StructuredLightModel& model, std::string_view name, core::Value value) {
  core::Tuple tuple;
  tuple.append(std::move(value));
  set_structured_light_model_param(model, name, tuple);
}

}  // namespace

void write_structured_light_model(const std::filesystem::path& path,
                                  const StructuredLightModel& model) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    throw core::Exception(core::Status::IoError, "write_structured_light_model",
                          "failed to open output model file");
  }
  write_bytes(out, kMagic.data(), kMagic.size());
  write_u32(out, kVersion);
  const auto& p = model.parameters();
  write_i64(out, p.pattern_width);
  write_i64(out, p.pattern_height);
  write_i64(out, p.min_stripe_width);
  write_i64(out, p.single_stripe_width);
  write_i64(out, p.min_gray_difference);
  write_u8(out, static_cast<std::uint8_t>(p.orientation));
  write_u8(out, static_cast<std::uint8_t>(p.pattern_type));
  write_u8(out, static_cast<std::uint8_t>(p.normalization));
  write_u8(out, p.persistence ? 1 : 0);

  write_u8(out,p.quality_output?1:0);write_real64(out,p.min_phase_amplitude);
  write_real64(out,p.intensity_stddev);write_real64(out,p.projector_coordinate_stddev);
  write_i64(out,p.hole_max_width);write_real64(out,p.hole_max_depth_jump);write_i64(out,p.saturation_margin);
  const bool has_setup = model.camera_setup().has_value();
  write_u8(out, has_setup ? 1 : 0);
  if (has_setup) {
    const auto& setup = *model.camera_setup();
    if (setup.kind() != CameraSetupKind::CameraProjector || !setup.complete()) {
      throw core::Exception(core::Status::InvalidState, "write_structured_light_model",
                            "structured-light camera setup is incomplete");
    }
    write_tuple(out, calibration::camera_parameters_to_tuple(setup.camera(0)));
    write_tuple(out, calibration::camera_parameters_to_tuple(setup.camera(1)));
    write_tuple(out, calibration::pose_to_halcon_tuple(setup.device_1_in_device_0_pose()));
  }
}

StructuredLightModel read_structured_light_model(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) io_error("failed to open input model file");
  try {
    char magic[8];
    read_bytes(in, magic, sizeof(magic));
    if (std::string_view(magic, sizeof(magic)) != kMagic) io_error("invalid model file signature");
    const auto version=read_u32(in);if(version<1 || version>kVersion)io_error("unsupported model file version");

    const auto width = read_i64(in);
    const auto height = read_i64(in);
    const auto min_stripe_width = read_i64(in);
    const auto single_stripe_width = read_i64(in);
    const auto min_gray_difference = read_i64(in);
    const auto orientation = read_u8(in);
    const auto pattern_type = read_u8(in);
    const auto normalization = read_u8(in);
    const auto persistence = read_u8(in);
    if (orientation > static_cast<std::uint8_t>(PatternOrientation::Both) ||
        pattern_type > static_cast<std::uint8_t>(StructuredLightPatternType::SingleStripe) ||
        normalization > static_cast<std::uint8_t>(StructuredLightNormalization::InvertedPattern) ||
        persistence > 1) {
      io_error("invalid structured-light parameter value");
    }

    auto model = create_structured_light_model("3d_reconstruction");
    set_value(model, "pattern_width", width);
    set_value(model, "pattern_height", height);
    set_value(model, "min_stripe_width", min_stripe_width);
    set_value(model, "min_gray_difference", min_gray_difference);
    set_value(model, "pattern_orientation",
              orientation_name(static_cast<PatternOrientation>(orientation)));
    set_value(model, "pattern_type",
              pattern_type_name(static_cast<StructuredLightPatternType>(pattern_type)));
    if (pattern_type == static_cast<std::uint8_t>(StructuredLightPatternType::SingleStripe)) {
      set_value(model, "single_stripe_width", single_stripe_width);
    }
    set_value(model, "normalization",
              normalization_name(static_cast<StructuredLightNormalization>(normalization)));
    set_value(model, "persistence", persistence != 0);

    if(version>=2) {
      const auto quality=read_u8(in);if(quality>1)io_error("invalid quality marker");
      set_value(model,"quality_output",quality?"true":"false");
      set_value(model,"min_phase_amplitude",read_real64(in));set_value(model,"intensity_stddev",read_real64(in));
      set_value(model,"projector_coordinate_stddev",read_real64(in));set_value(model,"hole_max_width",read_i64(in));
      set_value(model,"hole_max_depth_jump",read_real64(in));set_value(model,"saturation_margin",read_i64(in));
    }
    const auto has_setup = read_u8(in);
    if (has_setup > 1) io_error("invalid camera setup marker");
    if (has_setup != 0) {
      auto setup = CameraSetupModel::create("camera_projector");
      setup.set_camera(0, calibration::parse_camera_parameters(read_tuple(in)));
      setup.set_camera(1, calibration::parse_camera_parameters(read_tuple(in)));
      setup.set_device_1_in_device_0_pose(calibration::pose_from_halcon_tuple(read_tuple(in)));
      set_structured_light_camera_setup(model, setup);
    }
    if (in.peek() != std::char_traits<char>::eof()) io_error("unexpected trailing data in model file");
    return model;
  } catch (const core::Exception& ex) {
    if (ex.status() == core::Status::IoError) throw;
    throw core::Exception(core::Status::IoError, "read_structured_light_model",
                          "invalid structured-light model content");
  }
}

}  // namespace mv::reconstruction
