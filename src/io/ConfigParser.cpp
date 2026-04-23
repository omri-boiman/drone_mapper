#include "io/ConfigParser.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <algorithm>

namespace drone {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Trim leading/trailing whitespace
static std::string Trim(const std::string& s)
{
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return {};
    const auto b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

// Load a key=value file into a simple map; skip blank lines and # comments
using KVMap = std::vector<std::pair<std::string, std::string>>;

static KVMap LoadKV(const std::filesystem::path& filePath,
                    ErrorLogger& logger,
                    bool& opened)
{
    KVMap result;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open config file: " << filePath << "\n";
        opened = false;
        return result;
    }
    opened = true;

    int lineNum = 0;
    std::string line;
    while (std::getline(file, line)) {
        ++lineNum;
        const std::string t = Trim(line);
        if (t.empty() || t[0] == '#') continue;

        const auto eq = t.find('=');
        if (eq == std::string::npos) {
            logger.Log(filePath.filename().string() + " line " +
                       std::to_string(lineNum) + ": no '=' found — skipped");
            continue;
        }
        result.emplace_back(Trim(t.substr(0, eq)), Trim(t.substr(eq + 1)));
    }
    return result;
}

// Find a key in the KVMap; returns empty string if not found
static std::string Find(const KVMap& kv, const std::string& key)
{
    for (const auto& [k, v] : kv) {
        if (k == key) return v;
    }
    return {};
}

// Parse a double from string; on failure log and return defaultVal
static double GetDouble(const KVMap& kv, const std::string& key,
                        double defaultVal, const std::string& file,
                        ErrorLogger& logger)
{
    const std::string val = Find(kv, key);
    if (val.empty()) {
        logger.Log(file + ": missing key '" + key +
                   "' — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
    try {
        return std::stod(val);
    } catch (...) {
        logger.Log(file + ": bad value for '" + key + "' ('" + val +
                   "') — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
}

// Typed wrappers — parse then attach the correct unit
static Centi GetCenti(const KVMap& kv, const std::string& key,
                      Centi defaultVal, const std::string& file,
                      ErrorLogger& logger)
{
    const double raw = GetDouble(kv, key,
        defaultVal.numerical_value_in(si::centi<si::metre>), file, logger);
    return raw * si::centi<si::metre>;
}

static Degrees GetDegrees(const KVMap& kv, const std::string& key,
                          Degrees defaultVal, const std::string& file,
                          ErrorLogger& logger)
{
    const double raw = GetDouble(kv, key,
        defaultVal.numerical_value_in(si::degree), file, logger);
    return raw * si::degree;
}

// Parse an int from string; on failure log and return defaultVal
static int GetInt(const KVMap& kv, const std::string& key,
                  int defaultVal, const std::string& file,
                  ErrorLogger& logger)
{
    const std::string val = Find(kv, key);
    if (val.empty()) {
        logger.Log(file + ": missing key '" + key +
                   "' — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
    try {
        return std::stoi(val);
    } catch (...) {
        logger.Log(file + ": bad value for '" + key + "' ('" + val +
                   "') — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
}

// Parse boundary_polygon = (x1,y1),(x2,y2),...
static std::vector<std::pair<double,double>>
ParsePolygon(const std::string& raw, const std::string& file,
             ErrorLogger& logger)
{
    std::vector<std::pair<double,double>> poly;
    if (raw.empty()) {
        logger.Log(file + ": missing key 'boundary_polygon' — polygon will be empty");
        return poly;
    }

    std::string s = raw;
    // Remove all spaces
    s.erase(std::remove(s.begin(), s.end(), ' '), s.end());

    std::istringstream ss(s);
    std::string token;
    int idx = 0;
    while (std::getline(ss, token, ')')) {
        ++idx;
        if (token.empty()) continue;
        // Strip leading ',' then '(' (order matters: tokens after the first
        // arrive as ",(x,y" so we must peel the comma before the paren)
        if (!token.empty() && token.front() == ',') token = token.substr(1);
        if (!token.empty() && token.front() == '(') token = token.substr(1);

        const auto comma = token.find(',');
        if (comma == std::string::npos) {
            logger.Log(file + ": bad polygon vertex #" + std::to_string(idx) +
                       " '" + token + "' — skipped");
            continue;
        }
        try {
            const double x = std::stod(token.substr(0, comma));
            const double y = std::stod(token.substr(comma + 1));
            poly.emplace_back(x, y);
        } catch (...) {
            logger.Log(file + ": bad polygon vertex #" + std::to_string(idx) +
                       " '" + token + "' — skipped");
        }
    }

    if (poly.size() < 3) {
        logger.Log(file + ": boundary_polygon has fewer than 3 vertices — "
                          "mapping boundaries will be unreliable");
    }
    return poly;
}

// ---------------------------------------------------------------------------
// ParseDroneConfig
// ---------------------------------------------------------------------------

bool ParseDroneConfig(const std::filesystem::path& filePath,
                      DroneConfig&                 out,
                      ErrorLogger&                 logger)
{
    bool opened = false;
    const KVMap kv = LoadKV(filePath, logger, opened);
    if (!opened) return false;

    const std::string f = filePath.filename().string();

    out.minPassWidth  = GetCenti  (kv, "min_pass_width_cm",        out.minPassWidth,   f, logger);
    out.minPassLength = GetCenti  (kv, "min_pass_length_cm",       out.minPassLength,  f, logger);
    out.minPassHeight = GetCenti  (kv, "min_pass_height_cm",       out.minPassHeight,  f, logger);
    out.lidarFov      = GetDegrees(kv, "lidar_fov_deg",            out.lidarFov,       f, logger);
    out.lidarMinRange = GetCenti  (kv, "lidar_min_range_cm",       out.lidarMinRange,  f, logger);
    out.lidarMaxRange = GetCenti  (kv, "lidar_max_range_cm",       out.lidarMaxRange,  f, logger);
    out.lidarResAtDist1 = GetCenti(kv, "lidar_res_at_dist1_cm",   out.lidarResAtDist1, f, logger);
    out.lidarDist1      = GetCenti(kv, "lidar_dist1_cm",           out.lidarDist1,      f, logger);
    out.lidarResAtDist2 = GetCenti(kv, "lidar_res_at_dist2_cm",   out.lidarResAtDist2, f, logger);
    out.lidarDist2      = GetCenti(kv, "lidar_dist2_cm",           out.lidarDist2,      f, logger);
    out.maxRotate  = GetDegrees(kv, "max_rotate_deg",  out.maxRotate,  f, logger);
    out.maxAdvance = GetCenti  (kv, "max_advance_cm",  out.maxAdvance, f, logger);
    out.maxElevate = GetCenti  (kv, "max_elevate_cm",  out.maxElevate, f, logger);

    return true;
}

// ---------------------------------------------------------------------------
// ParseMissionConfig
// ---------------------------------------------------------------------------

bool ParseMissionConfig(const std::filesystem::path& filePath,
                        MissionConfig&               out,
                        ErrorLogger&                 logger)
{
    bool opened = false;
    const KVMap kv = LoadKV(filePath, logger, opened);
    if (!opened) return false;

    const std::string f = filePath.filename().string();

    out.boundaryPolygon     = ParsePolygon(Find(kv, "boundary_polygon"), f, logger);
    out.minHeight           = GetCenti(kv, "min_height_cm",  out.minHeight,  f, logger);
    out.maxHeight           = GetCenti(kv, "max_height_cm",  out.maxHeight,  f, logger);
    out.outputResXYDecimals = GetInt(kv, "output_resolution_xy_decimals",
                                     out.outputResXYDecimals, f, logger);
    out.outputResHDecimals  = GetInt(kv, "output_resolution_h_decimals",
                                     out.outputResHDecimals,  f, logger);
    out.startX      = GetCenti(kv, "start_x_cm",      out.startX,      f, logger);
    out.startY      = GetCenti(kv, "start_y_cm",      out.startY,      f, logger);
    out.startHeight = GetCenti(kv, "start_height_cm", out.startHeight, f, logger);

    return true;
}

} // namespace drone
