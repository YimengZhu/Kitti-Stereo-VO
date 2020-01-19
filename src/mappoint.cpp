#include "mappoint.h"

using namespace cv;
using namespace Eigen;

MapPoint::MapPoint()
    :id_(-1), pos_(Vector3d(0, 0, 0)), norm_(Vector3d(0, 0, 0)), observed_times_(0)
    {}

MapPoint::MapPoint(long id, Vector3d position, Vector3d norm)
    :id_(id), pos_(position), norm_(norm), observed_times_(0)
    {}
