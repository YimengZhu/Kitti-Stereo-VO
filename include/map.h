#ifndef MAP_H
#define MAP_H

#include "frame.h"
#include "mappoint.h"
#include "<unordered_map>"

class Map{

    public:
        typedef shared_ptr<Map> Ptr;
        
        unordered_map<unsigned long, MapPoint::Ptr> map_points_;

        unordered_map<unsigned long, Frame::Ptr> keyframes_;

        Map(){}

        void insertKeyFrame(Frame::Ptr frame);

        void insertMapPoint(MapPoint::Ptr map_point);
};

#endif
