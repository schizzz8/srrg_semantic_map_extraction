#pragma once
#include "srrg_types/cloud_3d.h"
//#include "srrg_types/defs.h"


class StructureAnalyzer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StructureAnalyzer();

    void compute(srrg_core::Cloud3D *cloud_, const Eigen::Isometry3f& iso=Eigen::Isometry3f::Identity());

    inline void setCellResolution(float cell_resolution_) { _cell_resolution = cell_resolution_; }
    inline float cellResolution() const { return _cell_resolution; }
    inline const srrg_core::IntImage& indices() const {return _indices;}
    inline const srrg_core::FloatImage& elevations() const {return _elevations; }
    inline const srrg_core::FloatImage& obstacles() const {return _obstacles; }
    inline const Eigen::Vector3f& upper() const { return _upper; }
    inline const Eigen::Vector3f& lower() const { return _lower; }
    inline const Eigen::Vector3f& bottom() const { return _bottom; }
    inline const Eigen::Vector3i& size() const { return _size;}
    inline srrg_core::UnsignedCharImage& classified() {return _classified; }
    inline const srrg_core::UnsignedCharImage& classified() const {return _classified; }

protected:
    srrg_core::IntImage _indices;
    srrg_core::FloatImage _elevations;
    srrg_core::FloatImage _obstacles;
    srrg_core::Cloud3D _transformed_cloud;
    float _cell_resolution;
    float _normal_cos_threshold;
    Eigen::Vector3f _upper;
    Eigen::Vector3f _lower;
    Eigen::Vector3f _bottom;
    Eigen::Vector3i _size;
    srrg_core::UnsignedCharImage _classified;
};
