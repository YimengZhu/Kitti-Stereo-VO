#ifndef FRAME_H
#define FRAME_H

#include <memory>
#include "camera.h"
#include <Eigen/Core>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <opencv2/core/core.hpp>

using namespace Sophus;
using namespace cv;

class Frame{
    public: 
        typedef std::shared_ptr<Frame> Ptr;
    
        unsigned long id_;
    
        double time_stamp_;
    
        SE3 T_c_w_;
    
        Camera::Ptr camera_;
    
        Mat image_left_, image_right_;
    
        bool is_key_frame_;
    
    
    
        Frame();
    
        Frame(long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr,
                Mat image_left=Mat(), Mat image_right=Mat());
    
        ~Frame();
    
        static Frame::Ptr createFrame();
    
        Vector3d getCamCenter() const;
    
        void setPose(const SE3& T_c_w);
    
        bool isInFrame(const Vector3d& p_w);

};

#endif
