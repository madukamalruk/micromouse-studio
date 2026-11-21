#pragma once
// Live robot config editor logic (v0.2.0, Tuning & Reports): an editable copy of the key
// RobotConfig numbers in friendly units, validation (including the 6 V motor rule), applying the
// edit back, and "Save copy" that never overwrites an existing file. No GUI dependencies.

#include <filesystem>
#include <string>
#include <vector>

#include "sim/core/robot_config.hpp"

namespace mmsim::app {

struct ConfigEdit {
    double mass_g = 100.0;
    double wheel_radius_mm = 16.0;
    double track_width_mm = 72.0;
    double max_voltage = 6.0;
    bool slip_enabled = false;
    double tyre_mu = 1.0;
    bool battery_model = false;
    double r_int_ohm = 0.15;
    double sensor_range_mm = 250.0;
    double noise_sigma = 0.0;    // raw units (mm for ToF, ADC counts for IR)
    double noise_rel_pct = 0.0;  // percent of the reading

    // The values the copy was made from: applying only touches fields the user changed, so a
    // config with different sensors is not flattened by accident.
    double orig_range_mm = 250.0, orig_sigma = 0.0, orig_rel_pct = 0.0;
    bool has_sensors = false;
};

enum class IssueLevel { Warn, Error };
struct ConfigIssue {
    IssueLevel level;
    std::string text;
};

/// The motor voltage limit of the hardware (N20 on a 2S pack): a hard rule of the project.
inline constexpr double kMotorVoltageLimit = 6.0;

ConfigEdit edit_from_config(const core::RobotConfig& c);
/// Writes the edit into `c`. Fields that were not changed keep their exact original value.
void apply_edit(const ConfigEdit& e, core::RobotConfig& c);
/// Errors block Apply; warnings (e.g. max_voltage above 6.0 V) do not.
std::vector<ConfigIssue> validate_edit(const ConfigEdit& e);
bool has_errors(const std::vector<ConfigIssue>& issues);

/// "<base>_edited", "<base>_edited2", ... : the first name whose .json does not exist in `dir`.
std::string unique_copy_stem(const std::filesystem::path& dir, const std::string& base);
/// Writes `c` as `<dir>/<stem>.json` (creating dir). Refuses to overwrite an existing file.
bool save_config_copy(const core::RobotConfig& c, const std::filesystem::path& dir,
                      const std::string& stem, std::filesystem::path& out, std::string& error);

}  // namespace mmsim::app
