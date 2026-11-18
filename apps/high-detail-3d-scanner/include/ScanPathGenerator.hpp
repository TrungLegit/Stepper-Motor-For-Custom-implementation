#pragma once

#include "ScannerKinematics.hpp"
#include <vector>
#include <string>
#include <cstdint>

namespace scanner {

/**
 * @brief Algorithmic path generator for multi-axis 3D photogrammetry and scanning.
 *
 * Supports:
 * - Fibonacci Sphere Lattice: Quasi-Monte Carlo golden ratio angle distribution
 *   for uniform solid angle camera coverage without polar distortion.
 * - Uniform Grid: Regular azimuth x elevation grid with serpentine routing.
 * - CSV Waypoint Importer/Exporter: Custom user-defined scanning trajectories.
 * - Path Optimization: Minimizes motor travel time and vibration.
 */
class ScanPathGenerator {
public:
    /**
     * @brief Generate Fibonacci Sphere lattice scan path.
     * @param n_poses Number of scan viewpoints (e.g. 56, 120, 240)
     * @param elevation_min_deg Minimum elevation angle above equator (deg)
     * @param elevation_max_deg Maximum elevation angle towards zenith (deg)
     * @param focus_z_mm Linear focus distance stage offset in mm
     * @return List of SphericalPose targets
     */
    static std::vector<SphericalPose> fibonacci(uint32_t n_poses,
                                                double elevation_min_deg = 5.0,
                                                double elevation_max_deg = 85.0,
                                                double focus_z_mm = 0.0);

    /**
     * @brief Generate uniform angular grid scan path with serpentine routing.
     * @param azimuth_steps Number of angular divisions around 360 degrees
     * @param elevation_steps Number of elevation tiers between min and max
     * @param elevation_min_deg Lowest elevation angle (deg)
     * @param elevation_max_deg Highest elevation angle (deg)
     * @param focus_layers_mm Focus distance offsets (e.g. {0.0} or focus-stack layers)
     * @param serpentine If true, alternates azimuth direction per elevation tier
     * @return List of SphericalPose targets
     */
    static std::vector<SphericalPose> uniformGrid(uint32_t azimuth_steps = 36,
                                                  uint32_t elevation_steps = 12,
                                                  double elevation_min_deg = 5.0,
                                                  double elevation_max_deg = 85.0,
                                                  const std::vector<double>& focus_layers_mm = {0.0},
                                                  bool serpentine = true);

    /**
     * @brief Parse scan waypoints from a CSV string.
     * CSV format: `azimuth_deg, elevation_deg, focus_z_mm`
     * Ignores header rows and lines starting with `#`.
     */
    static std::vector<SphericalPose> fromCsv(const std::string& csv_content);

    /**
     * @brief Load scan waypoints from a CSV file.
     */
    static std::vector<SphericalPose> fromCsvFile(const std::string& filepath);

    /**
     * @brief Export list of poses to CSV string.
     */
    static std::string toCsv(const std::vector<SphericalPose>& poses);

    /**
     * @brief Reorder poses to minimize total angular travel distance (Traveling Salesman heuristic).
     */
    static void optimizePath(std::vector<SphericalPose>& poses);
};

} // namespace scanner
