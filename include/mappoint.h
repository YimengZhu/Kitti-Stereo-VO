#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <Eigen/Core>
#include <opencv2/core/core.cpp>

using namespace Eigen;
using namespace cv;

class MapPoint{
    public:
        unsigned long id_;
        Vector3d pos_;
        Vector3d norm_;
        Mat descriptor_;
        int observed_time;
        int matched_time;

        MapPoint();
        MapPoint(long id, Vector3d position, Vector3d norm);

};

#endif
