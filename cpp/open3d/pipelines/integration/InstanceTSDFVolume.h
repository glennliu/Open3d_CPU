#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "open3d/geometry/PointCloud.h"
#include "open3d/pipelines/integration/ScalableTSDFVolume.h"

namespace open3d {
namespace pipelines {
namespace integration {

class UniformTSDFVolume;

class InstanceTSDFVolume : public ScalableTSDFVolume {
public:
    InstanceTSDFVolume(double voxel_length,
                       double sdf_trunc,
                       TSDFVolumeColorType color_type,
                       int volume_unit_resolution = 16,
                       int depth_sampling_stride = 4);
    ~InstanceTSDFVolume() override;

public:
    Eigen::Vector3i LocateVolumeUnit(const Eigen::Vector3d &point) {
        return Eigen::Vector3i((int)std::floor(point(0) / volume_unit_length_),
                               (int)std::floor(point(1) / volume_unit_length_),
                               (int)std::floor(point(2) / volume_unit_length_));
    };

    void extractVoxelCentroids();

    bool query_observed_points(const geometry::PointCloud cloud_scan,
                            geometry::PointCloud &cloud_observed, int min_points);

protected:
    int closet_classes_length;
    std::vector<std::string> label_measurements;

};

}
}
}
