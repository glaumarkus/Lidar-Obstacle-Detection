#ifndef LIDAR_PROCESSOR_H_
#define LIDAR_PROCESSOR_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <unordered_set>
#include "render/box.h"
#include "tree.h"
#include "ransac.h"
#include "clustering.h"

/*
Render Mode 0 = Org, 1 = Filter, 2 = Plane/Obs, 3 = Filtered Clusters Bounding Box
*/
const int RENDER_MODE  = 3;

/*
Filter Params
*/
const string DATASET = "../src/sensors/data/pcd/data_1";
const double FILTER_RESOLUTION = .2;
const Eigen::Vector4f CROPBOX_MIN = {-7,-6.5,-2,0};
const Eigen::Vector4f CROPBOX_MAX = {20,6.5,1.5,1};
const Eigen::Vector4f ROOF_MIN = {-2.5,-1.5,-2,0};
const Eigen::Vector4f ROOF_MAX = {2.6,1.5,2,1};

using namespace pcl;
using namespace std;

class lidar_processor{
public:

    lidar_processor();
    ~lidar_processor();


    void process_measurement(
        visualization::PCLVisualizer::Ptr& viewer){

        vector<boost::filesystem::path> paths (boost::filesystem::directory_iterator{DATASET}, boost::filesystem::directory_iterator{});
        sort(paths.begin(), paths.end());
        auto stream_idx = paths.begin();

        while (!viewer->wasStopped ())
        {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
            io::loadPCDFile<PointXYZ> ((*stream_idx).string(), *cloud);

            if (RENDER_MODE != 0){
                PointCloud<PointXYZ>::Ptr cloud_filter (new PointCloud<PointXYZ>);
                cloud_filter = filter(cloud);
                if (RENDER_MODE != 1){
                    pair<PointCloud<pcl::PointXYZ>::Ptr,PointCloud<pcl::PointXYZ>::Ptr> segments = ransac3D(cloud_filter);
                    if (RENDER_MODE != 2) {
                        renderPointCloud(viewer, segments.first, "road cloud", Color(0,1,0));
                        render_cluster(viewer,segments.second);
                    }
                    else {
                        renderPointCloud(viewer, segments.first, "road cloud", Color(0,1,0));
                        renderPointCloud(viewer, segments.second, "obs cloud", Color(1,0,0));
                    }
                }
                else
                    renderPointCloud(viewer, cloud_filter, "road cloud", Color(1,1,1));
            }
            else 
                renderPointCloud(viewer, cloud, "road cloud", Color(1,1,1));

            stream_idx++;
            if (stream_idx == paths.end())
                stream_idx = paths.begin();

            viewer->spinOnce ();
        } 
    }

    void renderPointCloud(visualization::PCLVisualizer::Ptr& viewer, const PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
    {

        viewer->addPointCloud<PointXYZ> (cloud, name);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    }


    PointCloud<PointXYZ>::Ptr filter(PointCloud<PointXYZ>::Ptr& cloud){

        PointCloud<PointXYZ>::Ptr cloud_v (new PointCloud<PointXYZ>);
        // Voxelgrid
        VoxelGrid<PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(FILTER_RESOLUTION, FILTER_RESOLUTION, FILTER_RESOLUTION);
        vg.filter(*cloud_v);

        PointCloud<PointXYZ>::Ptr cloud_c (new PointCloud<PointXYZ>);

        //Cropbox
        CropBox<PointXYZ> region(true);
        region.setMin(CROPBOX_MIN);
        region.setMax(CROPBOX_MAX);
        region.setInputCloud(cloud_v);
        region.filter(*cloud_c);

        //filter roof
        vector<int> indices;
        CropBox<PointXYZ> roof(true);
        roof.setMin(ROOF_MIN);
        roof.setMax(ROOF_MAX);
        roof.setInputCloud(cloud_c);
        roof.filter(indices);

        PointIndices::Ptr inliers {new PointIndices};
        for (int p:indices)
            inliers->indices.push_back(p);

        ExtractIndices<PointXYZ> extract;
        extract.setInputCloud(cloud_c);
        extract.setIndices(inliers);
        extract.setNegative (true);
        extract.filter(*cloud_c);

        return cloud_c;

    }
};


#endif 
