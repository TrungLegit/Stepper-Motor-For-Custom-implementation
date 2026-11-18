#pragma once

#include "protocol/MeshLabSchema.hpp"
#include <vector>
#include <string>
#include <cstdint>

namespace scanner {

/**
 * @brief Multi-format dataset metadata and camera pose exporter.
 *
 * Supports:
 * 1. MeshLab ASCII `.ply` (camera centers, orientation normals, frustum pyramids)
 * 2. COLMAP photogrammetry (`cameras.txt`, `images.txt`)
 * 3. RealityCapture reconstruction (`poses.csv`)
 * 4. Native JSON metadata (`poses.json`)
 */
class PoseExporter {
public:
    PoseExporter() = default;
    explicit PoseExporter(std::vector<motion_core::protocol::ScanPoseRecord> records);

    void addPose(const motion_core::protocol::ScanPoseRecord& record);
    void setPoses(std::vector<motion_core::protocol::ScanPoseRecord> records);
    const std::vector<motion_core::protocol::ScanPoseRecord>& getPoses() const noexcept {
        return m_records;
    }
    size_t size() const noexcept { return m_records.size(); }
    void clear() noexcept { m_records.clear(); }

    // Serialization string generators
    std::string toMeshLabPly(bool include_frustums = false, double frustum_scale_m = 0.02) const;
    std::string toColmapImagesTxt(uint32_t camera_id = 1) const;
    std::string toColmapCamerasTxt(uint32_t width = 3840,
                                   uint32_t height = 2160,
                                   double focal_length_px = 3000.0,
                                   uint32_t camera_id = 1) const;
    std::string toRealityCaptureCsv() const;
    std::string toJson(bool pretty = true) const;

    // File exporters
    bool saveMeshLabPly(const std::string& filepath, bool include_frustums = false) const;
    bool saveColmap(const std::string& output_dir,
                    uint32_t width = 3840,
                    uint32_t height = 2160,
                    double focal_length_px = 3000.0) const;
    bool saveRealityCaptureCsv(const std::string& filepath) const;
    bool saveJson(const std::string& filepath, bool pretty = true) const;

    /**
     * @brief Batch export all supported reconstruction formats into a destination folder.
     * Generates:
     * - `poses.ply`
     * - `colmap/images.txt`
     * - `colmap/cameras.txt`
     * - `rc_poses.csv`
     * - `poses.json`
     */
    bool exportAll(const std::string& output_dir) const;

private:
    std::vector<motion_core::protocol::ScanPoseRecord> m_records;
};

} // namespace scanner
