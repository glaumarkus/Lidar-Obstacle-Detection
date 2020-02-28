#include "camera.h"
#include <pcl/common/common.h>
#include "lidar_processor.h"


#define CAMERA_ANGLE XY;

using namespace pcl;
using namespace std;

int main (int argc, char** argv)
{

    visualization::PCLVisualizer::Ptr viewer (new visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = CAMERA_ANGLE;
    initCamera(setAngle, viewer);
    
    lidar_processor* l;
    l->process_measurement(viewer);

}
