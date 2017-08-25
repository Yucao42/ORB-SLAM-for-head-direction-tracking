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
#include<string>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include"Converter.h"

using namespace std;


int main(int ac, char **av)
{
    string voca,setting;
    // Visual vocabulary needed by SLAM
    voca="ORBvoc.bin";

    // Processing video
    string cap="/home/cy/Documents/2017_summer/gaze_tracking/set03/01-16-14_h02_m39_s13_Parking2_scene_wAudio.mov";
    
    // Camera intrinsics of Luo's camera, change it if needed
    setting="LUO.yaml";

   
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voca,setting,ORB_SLAM2::System::MONOCULAR,true);

    // Main loop 
    // Video reader in opencv
    cv::VideoCapture capt(cap);
    cv::Mat im;
    double timestamp = 0;

    // Frame number marks
    int value1 = 23616;
    int value2 = 57686;

    // Direction matrix
    cv::Mat cur,reference,current;
    
    // Direction vector: v[0] x angle, v[1] y angle, v[2] z angle, all in degrees
    cv::Vec3d RotationVec;

    // Write txt flag
    bool flag =false;
    int start_frame = 8000;//8000.....29260, 8000
    capt.set(CV_CAP_PROP_POS_FRAMES, start_frame);
    ofstream myfile;
    myfile.open ("result.txt");

    for(;;)
    {
        // Multithread synchronization
        usleep(50000);

        // Show current frame number, comment it if not wanted
        cout<<"Frame: "<<capt.get(CV_CAP_PROP_POS_FRAMES)<<endl;
        
        // Read frame
        capt >> im;
	    timestamp ++;
        if(im.empty())
        {
            cerr << endl << "No image available" << endl;
            return 1;
        }

        // Calculate and save current frame's direction to cur
        cur = SLAM.TrackMonocular(im,timestamp);
        if(cur.cols==4 && cur.rows ==4)current = cur.colRange(0,3).rowRange(0,3);

        //As in dataplot, frame number of value1 is set as the origin, so its direction is the reference direction
        if(capt.get(CV_CAP_PROP_POS_FRAMES) == value1+1)
        {        
            reference = current.clone(); 
            // Starts writing txt
            flag = true;
        }

        // Frame number should be no bigger than value2
        if(flag && capt.get(CV_CAP_PROP_POS_FRAMES)< (value2+1))
        {
            // Calculate relative transformation matrix and change it to angles
            RotationVec = ORB_SLAM2::Converter::dcm2euler(current*(reference.t()));
            // Print to screen and write to txt
            cout<<RotationVec[0]<<"     "<<RotationVec[1]<<"     "<<RotationVec[2]<<"."<<endl;
            myfile<<RotationVec[0]<<"     "<<RotationVec[1]<<"     "<<RotationVec[2]<<endl;;
        }

    }

    // Stop all threads
    SLAM.Shutdown();
    myfile.close();
    return 0;
}

