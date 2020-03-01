#include "camera.h"
#include <pcl/common/common.h>
#include "lidar_processor.h"

int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    lidar_processor* l;
    l->process_measurement(viewer);
}
