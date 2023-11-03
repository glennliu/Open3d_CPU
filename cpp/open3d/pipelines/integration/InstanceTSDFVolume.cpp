#include <unordered_set>

#include "open3d/geometry/PointCloud.h"
#include "open3d/utility/Logging.h"
#include "open3d/pipelines/integration/InstanceTSDFVolume.h"
#include "open3d/pipelines/integration/UniformTSDFVolume.h"


namespace open3d {
namespace pipelines {
namespace integration {

InstanceTSDFVolume::InstanceTSDFVolume(double voxel_length,
                                       double sdf_trunc,
                                       TSDFVolumeColorType color_type,
                                       int volume_unit_resolution,
                                       int depth_sampling_stride)
    : ScalableTSDFVolume(voxel_length,
                         sdf_trunc,
                         color_type,
                         volume_unit_resolution,
                         depth_sampling_stride) {}

InstanceTSDFVolume::~InstanceTSDFVolume() {}

bool InstanceTSDFVolume::query_observed_points(
    const geometry::PointCloud cloud_scan,
    geometry::PointCloud &cloud_observed, int min_points)
{
    for (size_t i = 0; i < cloud_scan.points_.size(); i++)
    {
        Eigen::Vector3d point = cloud_scan.points_[i];
        Eigen::Vector3i index = LocateVolumeUnit(point);

        auto &volume_unit = volume_units_[index];
        if (!volume_unit.volume_) continue;

        // const int res = volume_unit.volume_->resolution_;
        auto volume_points =volume_unit.volume_->ExtractVoxelPointCloud();
        // UniformTSDFVolume *volume_unit_ptr = volume_unit.volume_.get();

    }
    return true;
}

}
}
}
