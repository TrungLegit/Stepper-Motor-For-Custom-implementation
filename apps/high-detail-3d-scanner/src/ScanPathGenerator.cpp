#include "ScanPathGenerator.hpp"
#include <cmath>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <iomanip>

namespace scanner {

static constexpr double PI = 3.14159265358979323846;
static constexpr double GOLDEN_ANGLE_RAD = 2.39996322972865332; // pi * (3.0 - sqrt(5.0))
static constexpr double RAD_TO_DEG = 180.0 / PI;
static constexpr double DEG_TO_RAD = PI / 180.0;

std::vector<SphericalPose> ScanPathGenerator::fibonacci(uint32_t n_poses,
                                                        double elevation_min_deg,
                                                        double elevation_max_deg,
                                                        double focus_z_mm) {
    std::vector<SphericalPose> poses;
    if (n_poses == 0) {
        return poses;
    }

    const double min_el = std::clamp(elevation_min_deg, 0.0, 89.9);
    const double max_el = std::clamp(elevation_max_deg, min_el, 90.0);

    const double y_min = std::sin(min_el * DEG_TO_RAD);
    const double y_max = std::sin(max_el * DEG_TO_RAD);

    poses.reserve(n_poses);
    for (uint32_t i = 0; i < n_poses; ++i) {
        // Uniform solid-angle distribution across spherical cap
        const double y = y_min + (y_max - y_min) * (static_cast<double>(i) + 0.5) / static_cast<double>(n_poses);
        const double elevation = std::asin(std::clamp(y, -1.0, 1.0)) * RAD_TO_DEG;

        const double theta = static_cast<double>(i) * GOLDEN_ANGLE_RAD;
        double azimuth = std::fmod(theta * RAD_TO_DEG, 360.0);
        if (azimuth < 0.0) {
            azimuth += 360.0;
        }

        poses.push_back({azimuth, elevation, focus_z_mm});
    }

    return poses;
}

std::vector<SphericalPose> ScanPathGenerator::uniformGrid(uint32_t azimuth_steps,
                                                          uint32_t elevation_steps,
                                                          double elevation_min_deg,
                                                          double elevation_max_deg,
                                                          const std::vector<double>& focus_layers_mm,
                                                          bool serpentine) {
    std::vector<SphericalPose> poses;
    if (azimuth_steps == 0 || elevation_steps == 0) {
        return poses;
    }

    const double min_el = std::clamp(elevation_min_deg, 0.0, 90.0);
    const double max_el = std::clamp(elevation_max_deg, min_el, 90.0);

    const double el_step = (elevation_steps > 1) ? (max_el - min_el) / static_cast<double>(elevation_steps - 1) : 0.0;
    const double az_step = 360.0 / static_cast<double>(azimuth_steps);

    const auto& layers = focus_layers_mm.empty() ? std::vector<double>{0.0} : focus_layers_mm;

    for (double focus_z : layers) {
        for (uint32_t e = 0; e < elevation_steps; ++e) {
            const double el = min_el + static_cast<double>(e) * el_step;
            const bool reverse_az = serpentine && (e % 2 != 0);

            for (uint32_t a = 0; a < azimuth_steps; ++a) {
                const uint32_t az_idx = reverse_az ? (azimuth_steps - 1 - a) : a;
                const double az = static_cast<double>(az_idx) * az_step;
                poses.push_back({az, el, focus_z});
            }
        }
    }

    return poses;
}

std::vector<SphericalPose> ScanPathGenerator::fromCsv(const std::string& csv_content) {
    std::vector<SphericalPose> poses;
    std::istringstream stream(csv_content);
    std::string line;

    while (std::getline(stream, line)) {
        // Strip whitespace and carriage return
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        auto first_char = line.find_first_not_of(" \t");
        if (first_char == std::string::npos || line[first_char] == '#') {
            continue; // Skip comments and empty lines
        }

        std::stringstream ss(line);
        std::string part;
        std::vector<std::string> tokens;
        while (std::getline(ss, part, ',')) {
            // Trim token whitespace
            part.erase(0, part.find_first_not_of(" \t"));
            part.erase(part.find_last_not_of(" \t") + 1);
            tokens.push_back(part);
        }

        if (tokens.size() >= 2) {
            try {
                SphericalPose p;
                p.azimuth_deg = std::stod(tokens[0]);
                p.elevation_deg = std::stod(tokens[1]);
                p.focus_z_mm = (tokens.size() >= 3) ? std::stod(tokens[2]) : 0.0;
                poses.push_back(p);
            } catch (...) {
                // Ignore header text rows or malformed numbers
            }
        }
    }

    return poses;
}

std::vector<SphericalPose> ScanPathGenerator::fromCsvFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return {};
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return fromCsv(buffer.str());
}

std::string ScanPathGenerator::toCsv(const std::vector<SphericalPose>& poses) {
    std::ostringstream ss;
    ss << "# azimuth_deg, elevation_deg, focus_z_mm\n";
    ss << std::fixed << std::setprecision(4);
    for (const auto& p : poses) {
        ss << p.azimuth_deg << ", " << p.elevation_deg << ", " << p.focus_z_mm << "\n";
    }
    return ss.str();
}

void ScanPathGenerator::optimizePath(std::vector<SphericalPose>& poses) {
    if (poses.size() <= 2) {
        return;
    }

    // Sort poses into elevation bands (rounded to nearest 5 deg)
    // and within each band sort azimuth in alternating directions (serpentine)
    std::sort(poses.begin(), poses.end(), [](const SphericalPose& a, const SphericalPose& b) {
        const int band_a = static_cast<int>(std::round(a.elevation_deg / 5.0));
        const int band_b = static_cast<int>(std::round(b.elevation_deg / 5.0));
        if (band_a != band_b) {
            return band_a < band_b;
        }
        // In even bands sort azimuth ascending, in odd bands sort descending
        if (band_a % 2 == 0) {
            return a.azimuth_deg < b.azimuth_deg;
        } else {
            return a.azimuth_deg > b.azimuth_deg;
        }
    });
}

} // namespace scanner
