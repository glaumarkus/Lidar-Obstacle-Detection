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
#define CLUSTER_DISTANCE 0.33
#define CLUSTER_MIN 20
#define CLUSTER_MAX 5000

inline void log(string x){cout << x << endl;}

static void clusterHelper(
    int index, 
    const vector<vector<float>> points, 
    vector<int>& cluster, 
    vector<bool>& processed, 
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

static vector<vector<int>> euclideanCluster(const vector<vector<float>>& points, KdTree* tree)
{
    vector<vector<int>> clusters;
    vector<bool> processed(points.size(), false);

    int i = 0;

    while (i < points.size()) {
        if (processed[i]) {
            i++;
            continue;
        }

        vector<int> cluster;
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


void render_obs_box(visualization::PCLVisualizer::Ptr& viewer, PointCloud<PointXYZ>::Ptr cloud, string name){
    // Find bounding box for the road
    PointXYZ minPoint, maxPoint;
    getMinMax3D(*cloud, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;
  
  	float obj_length = maxPoint.x - minPoint.x;
    float obj_width = maxPoint.y - minPoint.y;
    float obj_height = maxPoint.z - minPoint.z;
  
    vector<int> c;
    // color based on properties
  	if (obj_length > 1.3 || obj_width > 1.3){
      //likely car
      c = {1,0,0};
    }
    else if((obj_length > .75 || obj_width > .75) && obj_height > 1.3){
      //likely bike / pedestrian
      c = {1,1,0};
    }
    else {
      //likely obstacle
      c = {0,0,1};
    }
  
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

    vector<PointCloud<PointXYZ>::Ptr> clusters;
    vector<std::vector<float>> pts;

    KdTree* tree = new KdTree;

    log("Starting Tree");

    for (int i = 0; i < obs->points.size(); i++) {
        auto pt = obs->points[i];
        pts.push_back(vector<float> {pt.x, pt.y, pt.z});
        tree->insert(vector<float> {pt.x, pt.y, pt.z}, i);
    }

    log("Starting Cluster");

    vector<vector<int>> clusterIndices = euclideanCluster(pts, tree);

    for (auto clusterIndex : clusterIndices) {
        PointCloud<PointXYZ>::Ptr obs_cluster (new PointCloud<PointXYZ>);
        for (auto pointIndex : clusterIndex) {
            obs_cluster->points.push_back (obs->points[pointIndex]);
        }
        obs_cluster->width = obs_cluster->points.size();
        obs_cluster->height = 1;
        obs_cluster->is_dense = true;

        clusters.push_back(obs_cluster);
    }

    // render all clusters
    
    int cluster_id = 0;
    for(auto cluster: clusters)
    {
        render_obs_box(viewer, cluster, "obs"+std::to_string(cluster_id));
        cluster_id++;
    }
    log("Found Clusters: " + to_string(cluster_id));
}

#endif