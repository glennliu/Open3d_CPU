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
typedef std::shared_ptr<geometry::PointCloud> PointCloudPtr;

class InstanceTSDFVolume : public ScalableTSDFVolume {
public:
    InstanceTSDFVolume(double voxel_length,
                       double sdf_trunc,
                       TSDFVolumeColorType color_type,
                       int volume_unit_resolution = 16,
                       int depth_sampling_stride = 4);
    ~InstanceTSDFVolume() override;

public:
    std::shared_ptr<geometry::PointCloud> ExtractWeightedPointCloud(const float min_weight=0.0);

    /// Generate scan cloud from depth image. And query observed voxel points from the volume.
    bool query_observed_points(const std::shared_ptr<geometry::PointCloud> &cloud_scan,
                            std::shared_ptr<geometry::PointCloud> &cloud_observed);
    

    //todo: inaccurate
    /// @brief  Get the centroid of all volume units origin.
    /// @return 
    Eigen::Vector3d get_centroid();

protected:
    Eigen::Vector3i LocateVolumeUnit(const Eigen::Vector3d &point) {
        return Eigen::Vector3i((int)std::floor(point(0) / volume_unit_length_),
                               (int)std::floor(point(1) / volume_unit_length_),
                               (int)std::floor(point(2) / volume_unit_length_));
    };

    Eigen::Vector3d GetNormalAt(const Eigen::Vector3d &p);

    double GetTSDFAt(const Eigen::Vector3d &p);

};

}
}
}
