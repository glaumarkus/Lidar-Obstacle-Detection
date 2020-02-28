#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include <vector>
#include <string> 
#include <pcl/common/common.h>
#include "tree.h"
#include "render/box.h"

using namespace std;
using namespace pcl;


// params
const float CLUSTER_DISTANCE = 0.33;
const int CLUSTER_MIN = 20;
const int CLUSTER_MAX = 5000;

inline void log(string x){std::cout << x << std::endl;}

static void clusterHelper(
    int index, 
    const std::vector<std::vector<float>>& points, 
    std::vector<int>& cluster, 
    std::vector<bool>& processed, 
    KdTree* tree) {

    processed[index] = true;
    cluster.push_back(index);
    std::vector<int> nearest = tree->search(points[index], CLUSTER_DISTANCE);

    for (int idx : nearest) {
        if (!processed[idx]) {
            clusterHelper(idx, points, cluster, processed, tree);
        }
    }
}

static std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    int i = 0;

    while (i < points.size()) {
        if (processed[i]) {
            i++;
            continue;
        }

        std::vector<int> cluster;
        clusterHelper(i, points, cluster, processed, tree);
        if (cluster.size() >= CLUSTER_MIN && cluster.size() <= CLUSTER_MAX) {
            clusters.push_back(cluster);
        } 
        else {
            for (int remove_index : cluster) {
                processed[remove_index] = false;
            }
        }
        i++;
    }

    return clusters;
}


void render_obs_box(visualization::PCLVisualizer::Ptr& viewer, PointCloud<PointXYZ>::Ptr& cloud, std::string name, std::vector<int> c){
    // Find bounding box for the road
    PointXYZ minPoint, maxPoint;
    getMinMax3D(*cloud, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z - 0.4;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;
  
  	// render 
    viewer->addPointCloud<PointXYZ> (cloud, name +"cloud");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 4, name +"cloud");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name +"cloud");
  
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, c[0], c[1],c[2], name);
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_REPRESENTATION, visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name); 
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_COLOR, c[0], c[1], 0, name);
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.5, name);
    
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, c[0], c[1], c[2], name + "fill");
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_REPRESENTATION, visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name + "fill"); 
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name + "fill");
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, 0.5, name + "fill");
}

void render_cluster(visualization::PCLVisualizer::Ptr& viewer, PointCloud<PointXYZ>::Ptr& obs){

    std::vector<PointCloud<PointXYZ>::Ptr> clusters;
    std::vector<std::vector<float>> pts;

    KdTree* tree = new KdTree;

    log("Starting Tree");

    for (int i = 0; i < obs->points.size(); i++) {
        auto pt = obs->points[i];
        pts.push_back(std::vector<float> {pt.x, pt.y, pt.z});
        tree->insert(std::vector<float> {pt.x, pt.y, pt.z}, i);
    }

    log("Starting Cluster");

    std::vector<std::vector<int>> clusterIndices = euclideanCluster(pts, tree);

    for (const auto& clusterIndex : clusterIndices) {
        PointCloud<PointXYZ>::Ptr obs_cluster (new PointCloud<PointXYZ>);
        for (const auto& pointIndex : clusterIndex) {
            obs_cluster->points.push_back (obs->points[pointIndex]);
        }
        obs_cluster->width = obs_cluster->points.size();
        obs_cluster->height = 1;
        obs_cluster->is_dense = true;

        clusters.push_back(obs_cluster);
    }

    // render all clusters
    int cluster_id = 0;
    std::vector<std::vector<int>> colors = {{1,0,0}, {0,0,1}, {1,1,0}, {1,0,1}, {0,1,1}};
    for(auto cluster: clusters)
    {
        render_obs_box(viewer, cluster, "obs"+std::to_string(cluster_id), colors[cluster_id%5]);
        cluster_id++;
    }
    log("Found Clusters: " + to_string(cluster_id));
}

#endif
