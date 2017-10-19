#include "structure_analyzer.h"
#include <iostream>

using namespace std;
using namespace srrg_core;

StructureAnalyzer::StructureAnalyzer() {
    _cell_resolution=0.05;
    _normal_cos_threshold=cos(20*M_PI/180.0);
    _upper = Eigen::Vector3f::Zero();
    _lower = Eigen::Vector3f::Zero();
}

void StructureAnalyzer::compute(Cloud3D* cloud_, const Eigen::Isometry3f& iso) {

    cerr << "Extracting ground level with structure analyzer" << endl;
    cerr << "\t>>resolution: " << _cell_resolution << endl;

    _indices.release();
    _elevations.release();
    _transformed_cloud.clear();
    if (! cloud_)
        return;
    _transformed_cloud=*cloud_;
    //_transformed_cloud.transformInPlace(iso);
    float ires=1./_cell_resolution;
    _transformed_cloud.computeBoundingBox(_lower, _upper);

    cerr << "\t>>origin: " << _lower.transpose() << endl;

    Eigen::Vector3f range = _upper - _lower;
    _size = (range*ires).cast<int>();
    int cols=_size.x();
    int rows=_size.y();

    cerr << "\t>>size: " << _size.transpose() << endl;

    _bottom = _lower;
    _bottom.z() = 0;
    _indices.create(rows,cols);
    _elevations.create(rows,cols);
    _classified.create(rows,cols);
    _indices=-1;
    _elevations=4;
    _classified=127;
    float _robot_climb_step=0.05;
    float _robot_height=1.0;

    // compute the elevatio of the surface
    for (size_t i=0; i<_transformed_cloud.size(); i++){
        const RichPoint3D& p = _transformed_cloud[i];
        float z = p.point().z();
        Eigen::Vector3f projected_point = (p.point() - _bottom)*ires;
        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();
        if (r>=rows || r<0)
            continue;
        if (c>=cols || r<0)
            continue;
        float &h=_elevations.at<float>(rows-r-1,c);
        int& idx=_indices.at<int>(rows-r-1,c);
        if (z<h) {
            h=z;
            idx=i;
        }
    }

    // mark the cells that are obstacles
    for (size_t i=0; i<_transformed_cloud.size(); i++){
        const RichPoint3D& p = _transformed_cloud[i];
        Eigen::Vector3f projected_point = (p.point() - _bottom)*ires;
        float z = p.point().z();

        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();

        //cerr << r << "," << c << " ";
        if (r>=rows || r<0)
            continue;
        if (c>=cols || r<0)
            continue;
        float &g=_elevations.at<float>(rows-r-1,c);
        int& idx=_indices.at<int>(rows-r-1,c);
        float min_obstacle_height=g+_robot_climb_step;
        float max_obstacle_height=g+_robot_height;

        if (z< min_obstacle_height) {
            continue;
        }
        if (z>max_obstacle_height)
            continue;

        idx=-2;
        g=z;
    }
    // fill in the invalid points
    for (int r=0; r<rows; r++)
        for (int c=0; c<cols; c++) {
            int idx = _indices.at<int>(r,c);
            if (idx==-1)
                continue;
            if (idx<-1){
                _classified.at<unsigned char>(r,c)=255; //obstacles
                continue;
            }
            _classified.at<unsigned char>(r,c)=0; //free
        }


    // clean the spurious points
    for (int r=1; r<rows-1; r++)
        for (int c=1; c<cols-1; c++) {
            unsigned char & cell=_classified.at<unsigned char>(r,c);
            if (cell!=255)
                continue;
            // seek for the 8 neighbors and isolate spurious points
            bool one_big=false;
            for (int rr=-1; rr<=1; rr++)
                for (int cc=-1; cc<=1; cc++) {
                    if (rr==0 && cc==0)
                        continue;
                    one_big |= _classified.at<unsigned char>(r+rr,c+cc)==255;
                }
            if (! one_big) {
                cell=0;
            }
        }

    // color the points (visualization only)
    for (size_t i=0; i<_transformed_cloud.size(); i++){
        const RichPoint3D& p = _transformed_cloud[i];
        Eigen::Vector3f projected_point = (p.point() - _bottom)*ires;
        float z = p.point().z();
        //compute row and column of the projection
        int r=projected_point.y();
        int c=projected_point.x();
        if (r>=rows || r<0)
            continue;
        if (c>=cols || r<0)
            continue;
        float &g=_elevations.at<float>(rows-r-1,c);
        float max_obstacle_height=g+_robot_height;
        int& idx=_indices.at<int>(rows-r-1,c);
        if (idx<-1 || z>max_obstacle_height) {
            cloud_->at(i)._rgb=Eigen::Vector3f(1,0,0); //obstacle
        }
        else {
            cloud_->at(i)._rgb=Eigen::Vector3f(0,1,0); //free
        }
    }
}

