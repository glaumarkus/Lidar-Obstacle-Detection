#ifndef FILTER_H
#define FILTER_H

#include <vector>
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr filter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float FILTER_RESOLUTION,
    Eigen::Vector4f CROPBOX_MIN,
    Eigen::Vector4f CROPBOX_MAX){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_v (new pcl::PointCloud<pcl::PointXYZ>);
    // Voxelgrid
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(FILTER_RESOLUTION, FILTER_RESOLUTION, FILTER_RESOLUTION);
    vg.filter(*cloud_v);

    //return cloud_v;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c (new pcl::PointCloud<pcl::PointXYZ>);

    //Cropbox
    pcl::CropBox<pcl::PointXYZ> region(true);
    region.setMin(CROPBOX_MIN);
    region.setMax(CROPBOX_MAX);
    region.setInputCloud(cloud_v);
    region.filter(*cloud_c);

    //return cloud_c

    Eigen::Vector4f ROOF_MIN = {-2.5,-1.5,-2,0};
    Eigen::Vector4f ROOF_MAX = {2.6,1.5,2,1};

    //filter roof
    std::vector<int> indices;
    pcl::CropBox<pcl::PointXYZ> roof(true);
    roof.setMin(ROOF_MIN);
    roof.setMax(ROOF_MAX);
    roof.setInputCloud(cloud_c);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int p:indices)
        inliers->indices.push_back(p);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_c);
    extract.setIndices(inliers);
    extract.setNegative (true);
    extract.filter(*cloud_c);

    return cloud_c;
}

#endif
