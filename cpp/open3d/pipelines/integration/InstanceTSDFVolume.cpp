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

std::shared_ptr<geometry::PointCloud> InstanceTSDFVolume::ExtractWeightedPointCloud(const float min_weight)
{
    auto pointcloud = std::make_shared<geometry::PointCloud>();
    double half_voxel_length = voxel_length_ * 0.5;
    float w0, w1, f0, f1;
    Eigen::Vector3f c0{0.0, 0.0, 0.0}, c1{0.0, 0.0, 0.0};
    for (const auto &unit : volume_units_) {
        if (unit.second.volume_) {
            const auto &volume0 = *unit.second.volume_;
            const auto &index0 = unit.second.index_;
            for (int x = 0; x < volume0.resolution_; x++) {
                for (int y = 0; y < volume0.resolution_; y++) {
                    for (int z = 0; z < volume0.resolution_; z++) {
                        Eigen::Vector3i idx0(x, y, z);
                        w0 = volume0.voxels_[volume0.IndexOf(idx0)].weight_;
                        f0 = volume0.voxels_[volume0.IndexOf(idx0)].tsdf_;
                        if (color_type_ != TSDFVolumeColorType::NoColor)
                            c0 = volume0.voxels_[volume0.IndexOf(idx0)]
                                         .color_.cast<float>();
                        if (w0 >= min_weight && f0 < 0.98f && f0 >= -0.98f) {
                            Eigen::Vector3d p0 =
                                    Eigen::Vector3d(half_voxel_length +
                                                            voxel_length_ * x,
                                                    half_voxel_length +
                                                            voxel_length_ * y,
                                                    half_voxel_length +
                                                            voxel_length_ * z) +
                                    index0.cast<double>() * volume_unit_length_;
                            for (int i = 0; i < 3; i++) {
                                Eigen::Vector3d p1 = p0;
                                Eigen::Vector3i idx1 = idx0;
                                Eigen::Vector3i index1 = index0;
                                p1(i) += voxel_length_;
                                idx1(i) += 1;
                                if (idx1(i) < volume0.resolution_) {
                                    w1 = volume0.voxels_[volume0.IndexOf(idx1)]
                                                 .weight_;
                                    f1 = volume0.voxels_[volume0.IndexOf(idx1)]
                                                 .tsdf_;
                                    if (color_type_ !=
                                        TSDFVolumeColorType::NoColor)
                                        c1 = volume0.voxels_[volume0.IndexOf(
                                                                     idx1)]
                                                     .color_.cast<float>();
                                } else {
                                    idx1(i) -= volume0.resolution_;
                                    index1(i) += 1;
                                    auto unit_itr = volume_units_.find(index1);
                                    if (unit_itr == volume_units_.end()) {
                                        w1 = 0.0f;
                                        f1 = 0.0f;
                                    } else {
                                        const auto &volume1 =
                                                *unit_itr->second.volume_;
                                        w1 = volume1.voxels_[volume1.IndexOf(
                                                                     idx1)]
                                                     .weight_;
                                        f1 = volume1.voxels_[volume1.IndexOf(
                                                                     idx1)]
                                                     .tsdf_;
                                        if (color_type_ !=
                                            TSDFVolumeColorType::NoColor)
                                            c1 = volume1.voxels_
                                                         [volume1.IndexOf(idx1)]
                                                                 .color_
                                                                 .cast<float>();
                                    }
                                }
                                if (w1 != 0.0f && f1 < 0.98f && f1 >= -0.98f &&
                                    f0 * f1 < 0) {
                                    float r0 = std::fabs(f0);
                                    float r1 = std::fabs(f1);
                                    Eigen::Vector3d p = p0;
                                    p(i) = (p0(i) * r1 + p1(i) * r0) /
                                           (r0 + r1);
                                    pointcloud->points_.push_back(p);
                                    if (color_type_ ==
                                        TSDFVolumeColorType::RGB8) {
                                        pointcloud->colors_.push_back(
                                                ((c0 * r1 + c1 * r0) /
                                                 (r0 + r1) / 255.0f)
                                                        .cast<double>());
                                    } else if (color_type_ ==
                                               TSDFVolumeColorType::Gray32) {
                                        pointcloud->colors_.push_back(
                                                ((c0 * r1 + c1 * r0) /
                                                 (r0 + r1))
                                                        .cast<double>());
                                    }
                                    // has_normal
                                    pointcloud->normals_.push_back(
                                            GetNormalAt(p));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return pointcloud;

}

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

Eigen::Vector3d InstanceTSDFVolume::GetNormalAt(const Eigen::Vector3d &p) {
    Eigen::Vector3d n;
    const double half_gap = 0.99 * voxel_length_;
    for (int i = 0; i < 3; i++) {
        Eigen::Vector3d p0 = p;
        p0(i) -= half_gap;
        Eigen::Vector3d p1 = p;
        p1(i) += half_gap;
        n(i) = GetTSDFAt(p1) - GetTSDFAt(p0);
    }
    return n.normalized();
}

double InstanceTSDFVolume::GetTSDFAt(const Eigen::Vector3d &p) {
    Eigen::Vector3d p_locate =
            p - Eigen::Vector3d(0.5, 0.5, 0.5) * voxel_length_;
    Eigen::Vector3i index0 = LocateVolumeUnit(p_locate);
    auto unit_itr = volume_units_.find(index0);
    if (unit_itr == volume_units_.end()) {
        return 0.0;
    }
    const auto &volume0 = *unit_itr->second.volume_;
    Eigen::Vector3i idx0;
    Eigen::Vector3d p_grid =
            (p_locate - index0.cast<double>() * volume_unit_length_) /
            voxel_length_;
    for (int i = 0; i < 3; i++) {
        idx0(i) = (int)std::floor(p_grid(i));
        if (idx0(i) < 0) idx0(i) = 0;
        if (idx0(i) >= volume_unit_resolution_)
            idx0(i) = volume_unit_resolution_ - 1;
    }
    Eigen::Vector3d r = p_grid - idx0.cast<double>();
    float f[8];
    for (int i = 0; i < 8; i++) {
        Eigen::Vector3i index1 = index0;
        Eigen::Vector3i idx1 = idx0 + shift[i];
        if (idx1(0) < volume_unit_resolution_ &&
            idx1(1) < volume_unit_resolution_ &&
            idx1(2) < volume_unit_resolution_) {
            f[i] = volume0.voxels_[volume0.IndexOf(idx1)].tsdf_;
        } else {
            for (int j = 0; j < 3; j++) {
                if (idx1(j) >= volume_unit_resolution_) {
                    idx1(j) -= volume_unit_resolution_;
                    index1(j) += 1;
                }
            }
            auto unit_itr1 = volume_units_.find(index1);
            if (unit_itr1 == volume_units_.end()) {
                f[i] = 0.0f;
            } else {
                const auto &volume1 = *unit_itr1->second.volume_;
                f[i] = volume1.voxels_[volume1.IndexOf(idx1)].tsdf_;
            }
        }
    }
    return (1 - r(0)) * ((1 - r(1)) * ((1 - r(2)) * f[0] + r(2) * f[4]) +
                         r(1) * ((1 - r(2)) * f[3] + r(2) * f[7])) +
           r(0) * ((1 - r(1)) * ((1 - r(2)) * f[1] + r(2) * f[5]) +
                   r(1) * ((1 - r(2)) * f[2] + r(2) * f[6]));
}

}
}
}
