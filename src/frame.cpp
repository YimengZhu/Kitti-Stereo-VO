#include <frame.h>

using namespace Sophus;
using namespace Eigen;
using namespace cv;

Frame::Frame():id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
    {}

Frame::Frame(long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, 
        Mat image_left, Mat image_right)
    :id_(id), 
     time_stamp_(time_stamp), 
     T_c_w_(T_c_w), camera_(camera), 
     image_left_(image_left), 
     image_right_(image_right), 
     is_key_frame_(false) {}

Frame::~Frame(){}

Frame::Ptr Frame::createFrame(){
    static long factor_id = 0;
    return Frame::Ptr(new Frame(factor_id++));
}

void Frame::setPose(const SE3& T_c_w){
    T_c_w_ = T_c_w;
}

Vector3d Frame::getCamCenter() const {
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame(const Vector3d& p_w){
    Vector3d p_c = camera_->world2camera(p_w, T_c_w_);
    if(p_c(2, 0) < 0) 
        return false;

    Vector2d pixel = camera_->world2pixel(p_w, T_c_w_);
        return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<image_left_.cols 
        && pixel(1,0)<image_left_.rows;
}
