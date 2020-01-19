#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "g2o_types.h"
#include "config.h"
#include <memory>
#include "map.h"
#include "ORBextractor.h"
#include <vector>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sophus/se3.h>
#include <sophus/so3.h>

using namespace Sophus;
using namespace cv;

class VisualOdometry{

    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        
        enum VOState{
            INITIALIZING = -1,
            OK = 0,
            LOST
        };

        VOState state_;

        SLAMMap::Ptr map_;

        Frame::Ptr ref_, curr_;

        vector<Point3f> pts_3d_ref_;

        ORBextractor *ORBextractorLeft_, *ORBextractorRight_;

        vector<cv::KeyPoint> keypoints_curr_left_, keypoints_curr_right_;

        Mat descriptors_curr_left_, descriptors_curr_right_, descriptors_ref_left_;
        
        vector<DMatch> feature_matches_;

        SE3 T_c_r_estimated_;

        int num_inliers_, num_lost_;

        int mnScaleLevels;

        float mfScaleFactor, mfLogScaleFactor;

        vector <float> mvScaleFactors, mvInvScaleFactors, mvLevelSigma2, mvInvLevelSigma2;

        float match_ratio_;

        int max_num_lost_, min_inliers_;

        double key_frame_min_rot_, key_frame_min_trans_;

        float mbf_, mb_;

        vector<float> mvuRight_, mvDepth_;

        VisualOdometry();

        ~VisualOdometry();

        bool addFrame(Frame::Ptr frame);

    protected:
        void extractORB_left(const Mat& imageleft);

        void extractORB_right(const Mat& imageright);

        void extractKeyPoints(const Mat& left, const Mat& right);

        void computeStereoMatches();

        void featureMatching();
        
        void setRef3DPoints();

        void poseEstimationPnP();

        void addKeyFrame();
    
        bool checkEstimatedPose(); 
    
        bool checkKeyFrame();
};

#endif
