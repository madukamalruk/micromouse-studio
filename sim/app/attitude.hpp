#pragma once
// Compass maths shared by the Attitude panel and the move tracker (no GUI dependencies).
//
// The simulator's yaw theta is radians counter-clockwise from +x (east), and the maze view puts
// north (+y) at the top. A compass reads clockwise from north, which is also how the firmware
// numbers its headings (NORTH = 0, EAST = 1, ...), so everything user-facing uses the bearing.

#include <cmath>

namespace mmsim::app {

inline constexpr double kPiD = 3.14159265358979323846;

/// Wraps degrees into [0, 360).
inline double wrap_deg360(double d) {
    d = std::fmod(d, 360.0);
    return d < 0.0 ? d + 360.0 : d;
}

/// Wraps degrees into (-180, 180].
inline double wrap_deg180(double d) {
    d = wrap_deg360(d);
    return d > 180.0 ? d - 360.0 : d;
}

/// Compass bearing in degrees [0, 360): 0 = north (+y), 90 = east (+x), clockwise.
inline double bearing_deg(double theta_rad) {
    return wrap_deg360(90.0 - theta_rad * 180.0 / kPiD);
}

/// Sim yaw (radians, CCW from +x) for a compass bearing in degrees.
inline double theta_of_bearing(double bearing) {
    return (90.0 - bearing) * kPiD / 180.0;
}

/// 0 = N, 1 = E, 2 = S, 3 = W: the quarter turn nearest to the bearing.
inline int quarter_of_bearing(double bearing) {
    return static_cast<int>(std::floor(wrap_deg360(bearing) / 90.0 + 0.5)) & 3;
}

/// "N", "NE", "E", ... (8-point rose) for a bearing.
inline const char* rose8(double bearing) {
    static const char* const n[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    return n[static_cast<int>(std::floor(wrap_deg360(bearing) / 45.0 + 0.5)) & 7];
}

/// "N", "E", "S", "W" for a quarter index (0..3).
inline const char* quarter_name(int q) {
    static const char* const n[] = {"N", "E", "S", "W"};
    return n[q & 3];
}

}  // namespace mmsim::app
