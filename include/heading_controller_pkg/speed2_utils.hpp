#ifndef HEADING_CONTROLLER_PKG__SPEED2_UTILS_HPP_
#define HEADING_CONTROLLER_PKG__SPEED2_UTILS_HPP_

#include <cmath>
#include <string>
#include <unordered_map>
#include <stdexcept>

// ==========================================
// Utility: Speed conversion maps and function
// ==========================================
// SPEED_CHART[u]         = factor to convert FROM unit u TO km/h
// SPEED_CHART_INVERSE[u] = factor to convert FROM km/h TO unit u
const std::unordered_map<std::string, float> SPEED_CHART = {
    {"km/h", 1.0f},
    {"m/s", 3.6f},
    {"mph", 1.609344f},
    {"knot", 1.852f}
};

const std::unordered_map<std::string, float> SPEED_CHART_INVERSE = {
    {"km/h", 1.0f},
    {"m/s", 0.277777778f},
    {"mph", 0.621371192f},
    {"knot", 0.539956803f}
};

inline float convert_speed(float speed, const std::string& unit_from, const std::string& unit_to) {
    if (SPEED_CHART.find(unit_from) == SPEED_CHART.end() ||
        SPEED_CHART_INVERSE.find(unit_to) == SPEED_CHART_INVERSE.end()) {
        throw std::invalid_argument("Incorrect 'unit_from' or 'unit_to' value.");
    }
    // Convert unit_from -> km/h using SPEED_CHART, then km/h -> unit_to using
    // SPEED_CHART_INVERSE. (Previously these two maps were swapped, which
    // silently produced an inverted conversion factor.)
    float converted = speed * SPEED_CHART.at(unit_from) * SPEED_CHART_INVERSE.at(unit_to);
    return std::round(converted * 1000.0f) / 1000.0f;
}

// ==========================================
// Mathematical & Helper Functions
// ==========================================
// Haversine great-circle distance between two lat/lon points, in meters.
// Kept in double precision throughout: lat/lon values already use most of a
// float's ~7 significant digits, and dLat/dLon are differences of two
// similarly-sized values, so computing in float risks catastrophic
// cancellation on the short, incremental distances this node measures
// between consecutive GPS fixes.
inline double get_distance(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371.0 * 1000.0; // Earth radius in meters

    double lat_start = lat1 * M_PI / 180.0;
    double lon_start = lon1 * M_PI / 180.0;
    double lat_end = lat2 * M_PI / 180.0;
    double lon_end = lon2 * M_PI / 180.0;

    double dLat = lat_end - lat_start;
    double dLon = lon_end - lon_start;

    double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
               std::cos(lat_start) * std::cos(lat_end) *
               std::sin(dLon / 2.0) * std::sin(dLon / 2.0);
    // Clamp for safety: floating point error can occasionally push `a`
    // a hair above 1.0, which would make sqrt(1.0 - a) negative -> NaN.
    if (a > 1.0) a = 1.0;
    if (a < 0.0) a = 0.0;
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return c * R;
}

#endif  // HEADING_CONTROLLER_PKG__SPEED2_UTILS_HPP_