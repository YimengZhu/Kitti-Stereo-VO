#include "vo.h"
#include "iostream" 
#include <utility>

using namespace std;

VisualOdometry::VisualOdometry()
    :state_(INITIALIZING), 
     ref_ (nullptr), 
     curr_(nullptr), 
     map_(new SLAMMap), 
     num_lost_(0),
     num_inliers_(0){
    match_ratio_        = Config::get<float>("match_ratio");
    max_num_lost_       = Config::get<float>("max_num_lost");
    min_inliers_        = Config::get<int>("min_inliers");
    key_frame_min_rot_  = Config::get<double>("keyframe_rotation");
    key_frame_min_trans_= Config::get<double>("keyframe_translation");
    mbf_                = Config::get<double>("Camera.mbf");
    float fx_           = Config::get<float>("Camera.fx");
    mb_                 = mbf_ / fx_;
    int nFeatures       = Config::get<int>("ORBextractor.nFeatures");
    float fScaleFactor  = Config::get<float>("ORBextractor.scaleFactor");
    int nLevels         = Config::get<int>("ORBextractor.nLevels");
    int fIniThFAST      = Config::get<int>("ORBextractor.iniThFAST");
    int fMinThFAST 	    = Config::get<int>("ORBextractor.minThFAST");
    ORBextractorLeft_	= new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    ORBextractorRight_  = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    mnScaleLevels       = ORBextractorLeft_->GetLevels();
    mfScaleFactor       = ORBextractorLeft_->GetScaleFactor();
    mfLogScaleFactor    = log(mfScaleFactor);
    mvScaleFactors      = ORBextractorLeft_->GetScaleFactors();
    mvInvScaleFactors   = ORBextractorLeft_->GetInverseScaleFactors();
    mvLevelSigma2       = ORBextractorLeft_->GetScaleSigmaSquares();
    mvInvLevelSigma2    = ORBextractorLeft_->GetInverseScaleSigmaSquares();
}


VisualOdometry::~VisualOdometry(){}


bool VisualOdometry::addFrame(Frame::Ptr frame){
    switch(state_){
        case INITIALIZING: {
            state_ = OK;
            curr_ = ref_ = frame;
            map_->insertKeyFrame (frame);
            extractKeyPoints(curr_->image_left_, curr_->image_right_);
            computeStereoMatches();
            setRef3DPoints();
            break;
        }
        case OK:{
            curr_ = frame;
            extractKeyPoints(curr_->image_left_, curr_->image_right_);
            featureMatching();
            poseEstimationPnP();
            
            if(checkEstimatedPose() == true) {
                num_lost_ = 0;
                curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;
                ref_=curr_;
                setRef3DPoints();
                if(checkKeyFrame() == true) addKeyFrame();
            } else{
                num_lost_++;
                if(num_lost_ > max_num_lost_) state_ = LOST;
                return false;
            }
            break;
        }
        case LOST:{
            cout << "vo has lost." << endl;  
            break;           
        }
    }
    return true;
}


void VisualOdometry::extractORB_left(const Mat& imageleft){
    (*ORBextractorLeft_)(imageleft,cv::Mat(), keypoints_curr_left_, descriptors_curr_left_);
}

void VisualOdometry::extractORB_right(const Mat& imageright){
    (*ORBextractorRight_)(imageright,cv::Mat(), keypoints_curr_right_, descriptors_curr_right_);
}

void VisualOdometry::extractKeyPoints(const Mat& left, const Mat& right){
    extractORB_left(left);
    extractORB_right(right);
}

void VisualOdometry::computeStereoMatches(){
    const int TH_HIGH = 100;
    const int TH_LOW = 50;

    mvuRight_ = vector<float>(keypoints_curr_left_.size(), -1.0f);
    mvDepth_ = vector<float>(keypoints_curr_left_.size(), -1.0f);

    const int thOrbDist = (TH_HIGH + TH_LOW) / 2;
    const int nRows = ORBextractorLeft_->mvImagePyramid[0].rows;

    vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());
    for(int i = 0; i < nRows; i++) vRowIndices[i].reserve(200);

    for(int i = 0; i < keypoints_curr_right_.size(); i++){
        const cv::KeyPoint &kp = keypoints_curr_right_[i];
        const float &kpY = kp.pt.y;
        const float r = 2.0f * mvScaleFactors[keypoints_curr_right_[i].octave];
        const int maxr = ceil(kpY + r);
        const int minr = floor(kpY - r);
        for(int yi = minr; yi <= maxr; yi++) vRowIndices[yi].push_back(i);
    }

    const float minZ = mb_, minD = 0, maxD = mbf_ / minZ;

    vector<pair<int, int>> vDistIdx;
    vDistIdx.reserve(keypoints_curr_left_.size());

    for(int i = 0; i < keypoints_curr_left_.size(); i++){
        const cv::KeyPoint &kpL = keypoints_curr_left_[i];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y, &uL = kpL.pt.x;
        
        const vector<size_t> &vCandidates = vRowIndices[vL];
        if(vCandidates.empty()) continue;

        const float minU = uL - maxD, maxU = uL - minD;
        if(maxU < 0) continue;

        int bestDist = TH_HIGH;
        size_t bestIdR = 0;

        const cv::Mat &dL = descriptors_curr_left_.row(i);

        for(size_t c = 0; c < vCandidates.size(); c++){
            const size_t iR = vCandidates[c];
            const cv::KeyPoint &kpR = keypoints_curr_right_[iR];

            if(kpR.octave < levelL - 1 || kpR.octave > levelL + 1) continue;

            const float &uR = kpR.pt.x;

            if(uR >= minU  && uR <= maxU){
                const cv::Mat &dR = descriptors_curr_right_.row(iR);
                const int dist = ORBextractor::DescriptorDistance(dL, dR);
                if(dist < bestDist){
                    bestDist = dist;
                    bestIdR = iR;
                }
            }
        }

        if(bestDist < thOrbDist){
            const float uR0 = keypoints_curr_right_[bestIdR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x * scaleFactor);
            const float scaledvL = round(kpL.pt.y * scaleFactor);
            const float scaleduR0 = round(uR0 * scaleFactor);
        
            const int w = 5;
            cv::Mat IL = ORBextractorLeft_->mvImagePyramid[kpL.octave].
                rowRange(scaledvL-w,scaledvL+w+1).
                colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);
        
            int bestDist = INT_MAX, BESTINCr = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2 * L + 1);

            const float iniu = scaleduR0 - w, endu = scaleduR0 + L + w + 1;
            if(iniu < 0 || endu >= ORBextractorRight_->mvImagePyramid[kpL.octave].cols)
                continue;
            
            for(int incR = -L; incR <= L; incR++){
                cv::Mat IR = ORBextractorRight_->mvImagePyramid[kpL.octave].
                    rowRange(scaledvL - w,scaledvL + w + 1).
                    colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                IR.convertTo(IR, CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows, IR.cols, CV_32F);
        
                float dist = cv::norm(IL, IR, cv::NORM_L1);
                if(dist < bestDist){
                    bestDist = dist;
                    bestIdR = incR;
                }

                vDists[L+incR] = dist;
            }           

            if(bestIdR == -L || bestIdR == L) continue;

            const float dist1 = vDists[L + bestIdR - 1];
            const float dist2 = vDists[L+bestIdR];
            const float dist3 = vDists[L+bestIdR + 1];
            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));
            if(deltaR < -1 || deltaR > 1) continue;

            float bestuR = mvScaleFactors[kpL.octave] *
                ((float)scaleduR0 + (float)bestIdR + deltaR);

            float disparity = (uL -bestuR);

            if(disparity >= minD && disparity < maxD){
                if(disparity <= 0){
                    disparity = 0.01;
                    bestuR = uL - 0.01;
                }
                mvDepth_[i]=mbf_/disparity;
                mvuRight_[i] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,i));
            }
        }
        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for(int i = vDistIdx.size() - 1; i >= 0; i--){
            if(vDistIdx[i].first < thDist){
                break;
            } else{
                mvuRight_[vDistIdx[i].second] = -1;
                mvDepth_[vDistIdx[i].second] = -1;
            }
        }
    }
}


void VisualOdometry::setRef3DPoints(){
    pts_3d_ref_.clear();
    descriptors_ref_left_ = Mat();
    for(size_t i = 0; i < keypoints_curr_left_.size(); i++){
        double depth = mvDepth_[i];
        if(depth > 0){
            Vector2d pixel(keypoints_curr_left_[i].pt.x, keypoints_curr_left_[i].pt.y);
            Vector3d p_c = ref_->camera_->pixel2camera(pixel, depth);
            pts_3d_ref_.push_back(cv::Point3f(p_c(0, 0), p_c(1, 0), p_c(2,0)));
            descriptors_ref_left_.push_back(descriptors_curr_left_.row(i));
        }
    }
}


void VisualOdometry::featureMatching(){
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    matcher.match(descriptors_ref_left_, descriptors_curr_left_, matches);
    float min_dis = std::min_element(matches.begin(), matches.end(),
            [](const cv::DMatch& m1, const cv::DMatch& m2){
                return m1.distance < m2.distance;
            })->distance;

    feature_matches_.clear();
    for(cv::DMatch& m:matches){
        if(m.distance < max<float>(min_dis * match_ratio_, 30))
            feature_matches_.push_back(m);
    }
    cout<<"good matches: "<<feature_matches_.size()<<endl;
}


void VisualOdometry::poseEstimationPnP(){
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    for (cv::DMatch m:feature_matches_){
        pts3d.push_back(pts_3d_ref_[m.queryIdx]);
        pts2d.push_back(keypoints_curr_left_[m.trainIdx].pt);
    }
    Mat K = (cv::Mat_<double>(3,3)<<
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0,0,1
    );

    Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
    
    cout << "g2o starts" << endl;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    //std::unique_ptr<Block::LinearSolverType> linearSolver(
    //        new g2o::LinearSolverDense<Block::PoseMatrixType>()
    //        );
    
    // Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
    Block* solver_ptr(new Block(std::move(linearSolver)));
    
    // g2o::OptimizationAlgorithmLevenberg* solver = 
    //     new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr)); 
    g2o::OptimizationAlgorithmLevenberg* solver = 
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
        T_c_r_estimated_.rotation_matrix(),
        T_c_r_estimated_.translation()
    ));
    optimizer.addVertex(pose);

    for (int i=0; i<inliers.rows; i++){
        int index = inliers.at<int>(i, 0);
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
        edge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    T_c_r_estimated_ = SE3(pose->estimate().rotation(), 
            pose->estimate().translation());
}

bool VisualOdometry::checkEstimatedPose(){
    if(num_inliers_ < min_inliers_) {
        cout << "reject because inlier is too small: "<< num_inliers_ <<endl;
        return false;
    }

    Sophus::Vector6d d = T_c_r_estimated_.log();
    if(d.norm() > 5.0) {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    
    return true;
}


bool VisualOdometry::checkKeyFrame(){
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d t = d.head<3>(), r = d.tail<3>();

    if(r.norm() > key_frame_min_rot_ || t.norm() > key_frame_min_trans_){
        return true;
    }
    return false;
}


void VisualOdometry::addKeyFrame(){
    cout << "adding a key frame" << endl;
    map_->insertKeyFrame(curr_);
}
