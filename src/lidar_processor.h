#ifndef LIDAR_PROCESSOR_H_
#define LIDAR_PROCESSOR_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <iostream> 
#include <string>  
#include <vector>

#include "params.h"
#include "filter.h"
#include "ransac.h"
#include "tree.h"
#include "clustering.h"
#include "render.h"

static std::string paramsFilePath = "../src/params.txt";

class lidar_processor{
public:

    lidar_processor();
    ~lidar_processor();

    void process_measurement(
        pcl::visualization::PCLVisualizer::Ptr& viewer){

        static PipelineParams params;

        while (!viewer->wasStopped ())
        {

            // start loop with loading params again
            if (!params.fromFile(paramsFilePath)) {
                std::cerr << "Error reading params file" << std::endl;
                return;
            }

            std::vector<boost::filesystem::path> paths (boost::filesystem::directory_iterator{params.DATASET}, boost::filesystem::directory_iterator{});
            sort(paths.begin(), paths.end());
            auto stream_idx = paths.begin();

            // loop through input files
            while (true){

                // remove last image
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();

                // initialize new pointcloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::io::loadPCDFile<pcl::PointXYZ> ((*stream_idx).string(), *cloud);

                // filter cloud input
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZ>);
                cloud_filter = filter(
                    cloud, 
                    params.FILTER_RESOLUTION, 
                    params.CROPBOX_MIN, 
                    params.CROPBOX_MAX);

                // segment cloud
                std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segments = ransac3D(
                    cloud_filter,
                    params.MAX_ITERATIONS,
                    params.DISTANCE_TOLERANCE);

                // display options
                if (params.render_input_cloud)
                    renderPointCloud(viewer, cloud, "org_cloud", Color(1,1,1));
                if (params.render_filtered_cloud)
                    renderPointCloud(viewer, cloud_filter, "filter_cloud", Color(1,1,1));
                if (params.render_plane)
                    renderPointCloud(viewer, segments.first, "road_cloud", Color(0,1,0));
                if (params.render_obstacles)
                    renderPointCloud(viewer, segments.second, "obs_cloud", Color(1,0,0));
                if (params.render_clusters)
                    render_cluster(
                        viewer,
                        segments.second,
                        params.CLUSTER_DISTANCE,
                        params.CLUSTER_MIN,
                        params.CLUSTER_MAX
                        );

                stream_idx++;
                if (stream_idx == paths.end())
                    break;

                viewer->spinOnce();
            }
        } 
    }
};

#endif 
