#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include "config.h"
#include "vo.h"
#include <orbextractor.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "usage: run_vo parameter_file" << endl;
        return 1;
    }
    Config::setParameterFile ( argv[1] );

    VisualOdometry::Ptr vo ( new myslam::VisualOdometry );
    
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps);    
    const int nImages = vstrImageLeft.size();  
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);    
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   
    
    Camera::Ptr camera ( new myslam::Camera );

    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);

	if ( imLeft.data==nullptr || imRight.data==nullptr )
            break;
	Frame::Ptr pFrame = myslam::Frame::createFrame();

	pFrame->camera_ = camera;
	pFrame->image_left_ = imLeft;
	pFrame->image_right_= imRight;
	pFrame->time_stamp_ = vTimestamps[ni];
	boost::timer timer;

	vo->addFrame ( pFrame );

    cout<<"VO costs time: "<<timer.elapsed()<<endl;
	cout << "*******************************" << endl;
	if ( vo->state_ == myslam::VisualOdometry::LOST )
        break;
	SE3 Tcw = pFrame->T_c_w_.inverse();

    }
    return 0;
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);

        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++){
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

