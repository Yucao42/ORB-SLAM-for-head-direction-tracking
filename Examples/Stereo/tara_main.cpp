
/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include<opencv2/opencv.hpp>
#include<boost/bind.hpp>
#include<opencv2/core/core.hpp>
#include<../include/System.h>

using namespace std;
using namespace cv;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(Mat& msgLeft,Mat& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    long long  timestamp=0;
};

void read1(VideoCapture &a, Mat &b){
    a>>b;
}

int main(int ac, char** av)
{
//    if(argc != 4)
//    {
//        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;

//        return 1;
//    }
 cout<< "Usage: number of camera && xml file name. Tara camera only" << endl;
    string voca,setting;
    voca="ORBvoc.bin";
int cap=ac>0?(av[1][0]-'0'):1;
    setting=ac>1?av[2]:"stereo_tara.yaml";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voca,setting,ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    if(1)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(setting, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }
/*********   sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));*/
    VideoCapture left(cap);
    cv::Mat src;vector<Mat> mv;
    while(1){
        left>>src;
cv::split(src,mv);
        

        igb.GrabStereo(mv[1],mv[2]);
//                waitKey(30);
        if(igb.timestamp>800)break;
//        boost::bind(&ImageGrabber::GrabStereo,&igb,imLeft,imRight)();
    }

    // Stop all threads
    SLAM.Shutdown();
    cout<<"system down"<<endl;
    // Save camera trajectory
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
//    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
//    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    return 0;
}

void ImageGrabber::GrabStereo(Mat& imLeft, Mat& imRight )
{
    cv::Mat imLeft1, imRight1;
    cv::remap(imLeft,imLeft1,M1l,M2l,cv::INTER_LINEAR);
         cv::remap(imRight,imRight1,M1r,M2r,cv::INTER_LINEAR);
         mpSLAM->TrackStereo(imLeft1,imRight1,timestamp);
    timestamp+=1;
}

