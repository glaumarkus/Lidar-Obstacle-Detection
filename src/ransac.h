#ifndef RANSAC_H_
#define RANSAC_H_

#include <vector>
#include <unordered_set>
#include <pcl/common/common.h>

using namespace std;
using namespace pcl;

#define MAX_ITERATIONS 40
#define DISTANCE_TOLERANCE 0.4


pair<PointCloud<PointXYZ>::Ptr, PointCloud<PointXYZ>::Ptr> ransac3D(PointCloud<PointXYZ>::Ptr cloud){

    srand(time(NULL));
    unordered_set<int> inliers;

    int idx = 0;

    while (idx < MAX_ITERATIONS){

        unordered_set<int> inliers_temp;

        while (inliers_temp.size() < 3){
            inliers_temp.insert(rand()%(cloud->points.size()));
        }

        auto itr = inliers_temp.begin();
        float x1 = cloud->points[*itr].x;
        float y1 = cloud->points[*itr].y;
        float z1 = cloud->points[*itr].z;
        itr++;
        float x2 = cloud->points[*itr].x;
        float y2 = cloud->points[*itr].y;
        float z2 = cloud->points[*itr].z;
        itr++;
        float x3 = cloud->points[*itr].x;
        float y3 = cloud->points[*itr].y;
        float z3 = cloud->points[*itr].z;

        double v1[3] = {x2-x1, y2-y1,z2-z1};
        double v2[3] = {x3-x1, y3-y1,z3-z1};

        double a = v1[1]*v2[2]-v1[2]*v2[1];
        double b = v1[2]*v2[0]-v1[0]*v2[2];
        double c = v1[0]*v2[1]-v1[1]*v2[0];
        double d = -(a*x1 + b*y1 + c*z1);

        int i = 0;
        for (auto p:cloud->points){

            float x = p.x;
            float y = p.y;
            float z = p.z;
            float dist = fabs(a * x + b * y + c * z + d) / sqrt(a*a + b*b + c*c);

            if (dist < DISTANCE_TOLERANCE)
                inliers_temp.insert(i);
            i++;   
        }
        if (inliers_temp.size() > inliers.size()){
            inliers = inliers_temp;
        }

        idx++;
    }

    PointCloud<PointXYZ>::Ptr cloud_road(new PointCloud<PointXYZ>());
    PointCloud<PointXYZ>::Ptr cloud_obs(new PointCloud<PointXYZ>());

    for(uint i = 0; i < cloud->points.size(); i++)
    {
        PointXYZ point = cloud->points[i];
        if(inliers.count(i))
            cloud_road->points.push_back(point);
        else
            cloud_obs->points.push_back(point);
    }

    return pair<PointCloud<PointXYZ>::Ptr, PointCloud<PointXYZ>::Ptr> {cloud_road,cloud_obs};

}

#endif