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
#include<iomanip>

#include<opencv2/core/core.hpp>

//darknet
#include"Thirdparty/darknet/src/yolo.h"
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <vector>

#include "ORBextractor.h"
#include "Thirdparty/darknet/src/yolo.h"
#include "Thirdparty/darknet/src/box.h"

using namespace std;
using namespace cv;

void LoadImages_robotcar(const string &strSequence, vector<string> &vstrImageFilenames);
void Draw(cv::Mat im, vector<cv::KeyPoint>keys, string title, vector<DetectedObject> objects, vector<string> name, int flag);

vector<string> names;
int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_robotcar path_to_settings path_to_yolo_settings path_to_sequence" << endl;
        return 1;
    }
    // Retrieve paths to images, load Image file path and time of every frame (second)
    vector<string> vstrImageFilenames;
    LoadImages_robotcar(string(argv[3]), vstrImageFilenames);
    int nImages = vstrImageFilenames.size();

    // ORB settings
    FileStorage fSettings(argv[1],cv::FileStorage::READ);
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    ORB_SLAM2::ORBextractor* pORBextractorLeft = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    ORB_SLAM2::ORBextractor* pORBextractorOrigin = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    Yolo yolo(argv[2]);
    vector<string> name =yolo.get_labels_();
    names = name;
    cv::Mat im, imGray;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        cout << "image path: " << vstrImageFilenames[ni] << endl;
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        vector<DetectedObject> objects;
        yolo.detect(im, objects);

        cvtColor(im,imGray,CV_RGB2GRAY);

        std::vector<cv::KeyPoint> vKeys, vKeysOrigin;
        cv::Mat Descriptors, DescriptorsOrigin;

        //cost the same time ~ 0.0770s
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        (*pORBextractorLeft)(imGray, cv::Mat(), vKeys, Descriptors, objects);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        (*pORBextractorOrigin)(imGray, cv::Mat(), vKeysOrigin, DescriptorsOrigin);
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

        cout << "--->>> time1: " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() << endl;
        cout << "--->>> time2: " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() << endl;

        cv::Mat imOrigin = im.clone();
        Draw(im, vKeys, "remove object", objects, names, 0);
        Draw(imOrigin, vKeysOrigin, "origin",objects, names, 1);
    }
    return 0;
}

void LoadImages_robotcar(const string &strPathToSequence, vector<string> &vstrImageFilenames)
{

    string strPrefixLeft = strPathToSequence + "imagePath.txt";
    ifstream pathFile(strPrefixLeft.c_str());
    if(!pathFile.is_open()){
        cerr << "Cannot open " << strPrefixLeft << endl;
        exit(0);
    }

    while(!pathFile.eof())
    {
        string s;
        getline(pathFile, s);
        if(!s.empty()){
            vstrImageFilenames.push_back(s);
        }
    }
    pathFile.close();
}

void Draw(Mat im, vector<KeyPoint> keys, string title, vector<DetectedObject> objects, vector<string> name, int flag)
{
    cv::namedWindow(title);
    const int n = keys.size();
    const float r = 5;

    for(int i=0;i<n;i++)
    {
        cv::Point2f pt1, pt2;
        pt1.x=keys[i].pt.x-r;
        pt1.y=keys[i].pt.y-r;
        pt2.x=keys[i].pt.x+r;
        pt2.y=keys[i].pt.y+r;

        cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
        cv::circle(im,keys[i].pt,2,cv::Scalar(0,255,0),-1);
    }
    for(int i = 0; i < objects.size(); i++)
    {
        DetectedObject& o = objects[i];
        cv::rectangle(im, o.bounding_box, cv::Scalar(255,0,255), 2);

        string class_name = name[o.object_class];

        char str[255];

        sprintf(str,"%s %%%.2f", class_name.c_str(), o.prob);
        cv::putText(im, str, cv::Point2f(o.bounding_box.x,o.bounding_box.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,255), 1);
    }
//   cv::Mat im_sized;
//   cv::Size cSize(im.cols/2, im.rows/2);
//   cv::resize(im, im_sized, cSize);
   cv::imshow(title,im);
   cv::waitKey(1);
}
