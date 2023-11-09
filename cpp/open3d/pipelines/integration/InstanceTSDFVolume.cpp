#include <unordered_set>

#include "open3d/geometry/PointCloud.h"
#include "open3d/utility/Logging.h"
#include "open3d/pipelines/integration/InstanceTSDFVolume.h"
#include "open3d/pipelines/integration/UniformTSDFVolume.h"
#include "open3d/pipelines/integration/MarchingCubesConst.h"


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
    const std::shared_ptr<geometry::PointCloud> &cloud_scan,
    std::shared_ptr<geometry::PointCloud> &cloud_observed)
{
    // std::unordered_set<Eigen::Vector3i,utility::hash_eigen<Eigen::Vector3i>> observed_units;

    for (size_t i = 0; i < cloud_scan->points_.size(); i++)
    {
        Eigen::Vector3d point = cloud_scan->points_[i];
        // const Eigen::Vector3i index = LocateVolumeUnit(point);
        // if (volume_units_.find(index)!=volume_units_.end())
        //     observed_units.insert(index);

        Eigen::Vector3d p_locate =
            point - Eigen::Vector3d(0.5, 0.5, 0.5) * voxel_length_;
        Eigen::Vector3i index0 = LocateVolumeUnit(p_locate);
        auto unit_itr = volume_units_.find(index0);
        if (unit_itr == volume_units_.end()) {
            continue;
        }
        const auto &volume0 = *unit_itr->second.volume_;
        Eigen::Vector3i idx0; // root voxel index
        Eigen::Vector3d p_grid =
                (p_locate - index0.cast<double>() * volume_unit_length_) /
                voxel_length_;  // point position in voxels
        for (int i = 0; i < 3; i++) {
            idx0(i) = (int)std::floor(p_grid(i));
            if (idx0(i) < 0) idx0(i) = 0;
            if (idx0(i) >= volume_unit_resolution_)
                idx0(i) = volume_unit_resolution_ - 1;
        }

        // iterate over neighbor voxels
        for (int i = 0; i < 8; i++) {
            float w0 = 0.0f;
            float f0 = 0.0f;
            Eigen::Vector3i idx1 = idx0 + shift[i];
            if (idx1(0) < volume_unit_resolution_ &&
                idx1(1) < volume_unit_resolution_ &&
                idx1(2) < volume_unit_resolution_) {
                w0 = volume0.voxels_[volume0.IndexOf(idx1)].weight_;
                f0 = volume0.voxels_[volume0.IndexOf(idx1)].tsdf_;
            } 
            else {
                Eigen::Vector3i index1 = index0;
                for (int j = 0; j < 3; j++) {
                    if (idx1(j) >= volume_unit_resolution_) {
                        idx1(j) -= volume_unit_resolution_;
                        index1(j) += 1;
                    }
                }
                auto unit_itr1 = volume_units_.find(index1);
                if (unit_itr1 != volume_units_.end()) {
                    const auto &volume1 = *unit_itr1->second.volume_;
                    w0 = volume1.voxels_[volume1.IndexOf(idx1)].weight_;
                    f0 = volume1.voxels_[volume1.IndexOf(idx1)].tsdf_;
                }
            }
            if(w0!=0.0f && f0<0.98f && f0>=-0.98f){
                cloud_observed->points_.push_back(point);
                break;
            }
        }

    }
    return true;

    // Extract all the points in the observed units
    // if (observed_units.size() < min_units) return false;
    // else{
    //     cloud_observed = std::make_shared<geometry::PointCloud>();
    //     for (const auto &unit_id:observed_units){
    //         const auto &observed_unit = volume_units_[unit_id];
    //         if(!observed_unit.volume_) continue;
    //         *cloud_observed += *(observed_unit.volume_->ExtractVoxelPointCloud());
    //     }
    //     return true;
    // }
}

Eigen::Vector3d InstanceTSDFVolume::get_centroid()
{
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    int count = 0;
    for(const auto&unit:volume_units_){
        if(unit.second.volume_){
            centroid += unit.second.volume_->origin_;
            count++;
        }
    }

    if(count>0) return centroid/count;
    else return Eigen::Vector3d::Zero();

}

}
}
}
