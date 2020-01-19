#ifndef MAP_H
#define MAP_H

#include <memory>
#include "frame.h"
#include "mappoint.h"
#include <unordered_map>

class SLAMMap{

    public:
        typedef shared_ptr<SLAMMap> Ptr;
        
        unordered_map<unsigned long, MapPoint::Ptr> map_points_;

        unordered_map<unsigned long, Frame::Ptr> keyframes_;

        SLAMMap(){}

        void insertKeyFrame(Frame::Ptr frame);

        void insertMapPoint(MapPoint::Ptr map_point);
};

#endif
