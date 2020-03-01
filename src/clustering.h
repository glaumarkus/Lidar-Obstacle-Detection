#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include <vector>
#include <string> 
#include <pcl/common/common.h>
#include "tree.h"
#include "render.h"
#include "camera.h"

inline void log(std::string x){std::cout << x << std::endl;}

static void clusterHelper(
    int index, 
    const std::vector<std::vector<float>>& points, 
    std::vector<int>& cluster, 
    std::vector<bool>& processed, 
    KdTree* tree,
    float& CLUSTER_DISTANCE) {

    processed[index] = true;
    cluster.push_back(index);
    std::vector<int> nearest = tree->search(points[index], CLUSTER_DISTANCE);

    for (int idx : nearest) {
        if (!processed[idx]) {
            clusterHelper(idx, points, cluster, processed, tree, CLUSTER_DISTANCE);
        }
    }
}

static std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points, 
    KdTree* tree, 
    int CLUSTER_MIN,
    int CLUSTER_MAX,
    float CLUSTER_DISTANCE)
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
        clusterHelper(i, points, cluster, processed, tree, CLUSTER_DISTANCE);
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

void render_cluster(
    pcl::visualization::PCLVisualizer::Ptr& viewer, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr& obs,
    float CLUSTER_DISTANCE,
    int CLUSTER_MIN,
    int CLUSTER_MAX){

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::vector<std::vector<float>> pts;

    KdTree* tree = new KdTree;

    log("Starting Tree");

    for (int i = 0; i < obs->points.size(); i++) {
        auto pt = obs->points[i];
        pts.push_back(std::vector<float> {pt.x, pt.y, pt.z});
        tree->insert(std::vector<float> {pt.x, pt.y, pt.z}, i);
    }

    log("Starting Cluster");

    std::vector<std::vector<int>> clusterIndices = euclideanCluster(pts, tree, CLUSTER_MIN, CLUSTER_MAX, CLUSTER_DISTANCE);

    for (const auto& clusterIndex : clusterIndices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr obs_cluster (new pcl::PointCloud<pcl::PointXYZ>);
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

    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0), Color(0,1,1), Color(1,0,1)};
    for(auto cluster: clusters)
    {
        render_obs_box(viewer, cluster, "obs"+std::to_string(cluster_id), colors[cluster_id%5]);
        cluster_id++;
    }
    log("Found Clusters: " + std::to_string(cluster_id));
}

#endif