#ifndef HEADING_CONTROLLER_PKG__GNSS_CHECKPOINT_MONITOR_UTILS_HPP_
#define HEADING_CONTROLLER_PKG__GNSS_CHECKPOINT_MONITOR_UTILS_HPP_

#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <istream>

// ==========================================
// Utility math (Haversine & bearing)
// ==========================================
inline double haversine_m(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0;
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    if (a > 1.0) a = 1.0;
    if (a < 0.0) a = 0.0;
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return R * c;
}

inline double bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    double lat1r = lat1 * M_PI / 180.0;
    double lat2r = lat2 * M_PI / 180.0;
    double dlonr = (lon2 - lon1) * M_PI / 180.0;
    double y = std::sin(dlonr) * std::cos(lat2r);
    double x = std::cos(lat1r) * std::sin(lat2r) - std::sin(lat1r) * std::cos(lat2r) * std::cos(dlonr);
    double brng = std::atan2(y, x) * 180.0 / M_PI;
    double wrapped = std::fmod(brng, 360.0);
    if (wrapped < 0.0) wrapped += 360.0;
    return wrapped;
}

// Present but currently unused by the monitor's own logic, mirroring the
// Python (heading checks are fully delegated to an external aligner node).
inline double angle_diff_short(double target_deg, double current_deg) {
    double d = std::fmod(target_deg - current_deg + 180.0, 360.0);
    if (d < 0.0) d += 360.0;
    return d - 180.0;
}

// ==========================================
// Mirrors Python's `a or b or c` truthiness fallback chain, INCLUDING the
// quirk that an explicit value of 0 is treated as "not set" and falls
// through. Faithfully preserves the original's behavior (and its bug)
// rather than silently fixing it.
// ==========================================
inline double truthy_or(std::optional<double> a, std::optional<double> b, double fallback) {
    if (a.has_value() && a.value() != 0.0) return a.value();
    if (b.has_value() && b.value() != 0.0) return b.value();
    return fallback;
}

inline double truthy_or_single(std::optional<double> a, double fallback) {
    if (a.has_value() && a.value() != 0.0) return a.value();
    return fallback;
}

// ==========================================
// Checkpoint data
// ==========================================
struct Checkpoint {
    int id = 0;
    std::string name;
    double lat = 0.0;
    double lon = 0.0;
    double tolerance_m = 2.0;
    std::optional<double> desired_heading_deg;
    std::string behavior = "align_then_continue";
    bool visited = false;
    double last_visited_time = 0.0;
};

// ==========================================
// CSV checkpoint parser. Header row must contain column names (order-
// independent); recognized columns: id, name, lat/latitude, lon/longitude,
// tolerance_m, desired_heading_deg, behavior. Mirrors load_checkpoints()'s
// CSV branch, including the truthy-or-zero fallback quirks above.
// ==========================================
inline std::vector<Checkpoint> parse_checkpoints_csv(std::istream& stream, double tol_default) {
    std::vector<Checkpoint> checkpoints;

    std::string header_line;
    if (!std::getline(stream, header_line)) return checkpoints;

    std::vector<std::string> columns;
    {
        std::stringstream ss(header_line);
        std::string col;
        while (std::getline(ss, col, ',')) columns.push_back(col);
    }

    std::string line;
    int row_index = 0;
    while (std::getline(stream, line)) {
        std::vector<std::string> values;
        std::stringstream ss(line);
        std::string val;
        while (std::getline(ss, val, ',')) values.push_back(val);

        std::unordered_map<std::string, std::string> row;
        for (size_t i = 0; i < columns.size() && i < values.size(); ++i) {
            row[columns[i]] = values[i];
        }

        auto get_double = [&](const std::string& key) -> std::optional<double> {
            auto it = row.find(key);
            if (it == row.end() || it->second.empty()) return std::nullopt;
            try {
                return std::stod(it->second);
            } catch (...) {
                return std::nullopt;
            }
        };

        Checkpoint cp;
        cp.lat = truthy_or(get_double("lat"), get_double("latitude"), 0.0);
        cp.lon = truthy_or(get_double("lon"), get_double("longitude"), 0.0);

        auto id_it = row.find("id");
        cp.id = (id_it != row.end() && !id_it->second.empty()) ? std::stoi(id_it->second) : row_index;

        auto name_it = row.find("name");
        cp.name = (name_it != row.end() && !name_it->second.empty())
                      ? name_it->second
                      : ("pt_" + std::to_string(cp.id));

        cp.tolerance_m = truthy_or_single(get_double("tolerance_m"), tol_default);
        cp.desired_heading_deg = get_double("desired_heading_deg");

        auto behavior_it = row.find("behavior");
        cp.behavior = (behavior_it != row.end() && !behavior_it->second.empty())
                          ? behavior_it->second
                          : "align_then_continue";

        cp.visited = false;
        cp.last_visited_time = 0.0;

        checkpoints.push_back(cp);
        row_index++;
    }
    return checkpoints;
}

// ==========================================
// Checkpoint selection ("sequential" or "closest" among unvisited)
// ==========================================
inline std::optional<size_t> select_next_checkpoint(const std::vector<Checkpoint>& checkpoints,
                                                      const std::string& mode,
                                                      double gps_lat, double gps_lon) {
    std::vector<size_t> unvisited;
    for (size_t i = 0; i < checkpoints.size(); ++i) {
        if (!checkpoints[i].visited) unvisited.push_back(i);
    }
    if (unvisited.empty()) return std::nullopt;

    if (mode == "sequential") {
        return unvisited.front();
    }

    // "closest" mode
    std::optional<size_t> best;
    double best_d = 0.0;
    for (size_t i : unvisited) {
        double d = haversine_m(gps_lat, gps_lon, checkpoints[i].lat, checkpoints[i].lon);
        if (!best.has_value() || d < best_d) {
            best = i;
            best_d = d;
        }
    }
    return best;
}

// ==========================================
// Core tolerance-zone / pass-detection decision logic, extracted from
// main_loop(). Excludes wall-clock-based alignment-publish rate limiting,
// which stays in the node (it's a timing side effect, not a decision).
// ==========================================
enum class LoopAction {
    None,
    EnteredTolerance,
    RequestAlignment,
    ClearAndAdvance,
    MarkPassedAndAdvance
};

struct LoopDecision {
    LoopAction action = LoopAction::None;
    bool point_flag_next = false;
};

inline LoopDecision decide_loop_action(const Checkpoint& active, double distance_m,
                                        bool point_flag, bool alignment_done,
                                        std::optional<double> last_dist,
                                        double max_pass_distance) {
    if (distance_m <= active.tolerance_m) {
        if (!point_flag) {
            return {LoopAction::EnteredTolerance, true};
        }
        if (active.desired_heading_deg.has_value() && !alignment_done) {
            return {LoopAction::RequestAlignment, true};
        }
        return {LoopAction::None, true};
    }

    if (point_flag) {
        // Left tolerance after being inside -- Python calls
        // clear_and_advance() on BOTH the "aligned" and "not aligned"
        // paths here, so both collapse to the same action.
        return {LoopAction::ClearAndAdvance, false};
    }

    if (last_dist.has_value() &&
        distance_m > (active.tolerance_m + max_pass_distance) &&
        distance_m > last_dist.value() + 0.1) {
        return {LoopAction::MarkPassedAndAdvance, false};
    }

    return {LoopAction::None, false};
}

#endif  // HEADING_CONTROLLER_PKG__GNSS_CHECKPOINT_MONITOR_UTILS_HPP_