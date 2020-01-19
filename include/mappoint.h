#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <memory>

using namespace Eigen;
using namespace cv;

class MapPoint{
    public:
        typedef std::shared_ptr<MapPoint> Ptr;
        unsigned long id_;
        Vector3d pos_;
        Vector3d norm_;
        Mat descriptor_;
        int observed_times_;
        int matched_times_;

        MapPoint();
        MapPoint(long id, Vector3d position, Vector3d norm);

};

#endif
