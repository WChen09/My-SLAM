///**
//* This file is part of ORB-SLAM2.
//*
//* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
//* For more information see <https://github.com/raulmur/ORB_SLAM2>
//*
//* ORB-SLAM2 is free software: you can redistribute it and/or modify
//* it under the terms of the GNU General Public License as published by
//* the Free Software Foundation, either version 3 of the License, or
//* (at your option) any later version.
//*
//* ORB-SLAM2 is distributed in the hope that it will be useful,
//* but WITHOUT ANY WARRANTY; without even the implied warranty of
//* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//* GNU General Public License for more details.
//*
//* You should have received a copy of the GNU General Public License
//* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
//*/


//#include<iostream>
//#include<algorithm>
//#include<fstream>
//#include<chrono>
//#include<iomanip>

//#include<opencv2/core/core.hpp>

////darknet
//#include"Thirdparty/darknet/src/yolo.h"
//#include <string>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <fstream>
//#include <vector>

//#include "ORBextractor.h"
//#include "Thirdparty/darknet/src/yolo.h"
//#include "Thirdparty/darknet/src/box.h"

//using namespace std;
//using namespace cv;

//void LoadImages_robotcar(const string &strSequence, vector<string> &vstrImageFilenames);
//void Draw(cv::Mat im, vector<cv::KeyPoint>keys, string title, vector<DetectedObject> objects, vector<string> name, int flag);

//vector<string> names;
//int main(int argc, char **argv)
//{
//    if(argc != 4)
//    {
//        cerr << endl << "Usage: ./mono_robotcar path_to_settings path_to_yolo_settings path_to_sequence" << endl;
//        return 1;
//    }
//    // Retrieve paths to images, load Image file path and time of every frame (second)
//    vector<string> vstrImageFilenames;
//    LoadImages_robotcar(string(argv[3]), vstrImageFilenames);
//    int nImages = vstrImageFilenames.size();

//    // ORB settings
//    FileStorage fSettings(argv[1],cv::FileStorage::READ);
//    int nFeatures = fSettings["ORBextractor.nFeatures"];
//    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
//    int nLevels = fSettings["ORBextractor.nLevels"];
//    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
//    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
//    ORB_SLAM2::ORBextractor* pORBextractorLeft = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
//    ORB_SLAM2::ORBextractor* pORBextractorOrigin = new ORB_SLAM2::ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

//    Yolo yolo;
//    yolo.readConfig(argv[2]);
////    yolo.loadConfig();
//    vector<string> name =yolo.get_labels_();
//    names = name;
//    cv::Mat im, imGray;
//    for(int ni=0; ni<nImages; ni++)
//    {
//        // Read image from file
//        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
//        cout << "image path: " << vstrImageFilenames[ni] << endl;
//        if(im.empty())
//        {
//            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
//            return 1;
//        }

//        vector<DetectedObject> objects;
//        yolo.detect(im, objects);

//        cvtColor(im,imGray,CV_RGB2GRAY);

//        std::vector<cv::KeyPoint> vKeys, vKeysOrigin;
//        cv::Mat Descriptors, DescriptorsOrigin;

//        //cost the same time ~ 0.0770s
//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        (*pORBextractorLeft)(imGray, cv::Mat(), vKeys, Descriptors, objects, 0);
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//        (*pORBextractorOrigin)(imGray, cv::Mat(), vKeysOrigin, DescriptorsOrigin);
//        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

//        cout << "--->>> time1: " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() << endl;
//        cout << "--->>> time2: " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() << endl;

//        cv::Mat imOrigin = im.clone();
//        Draw(im, vKeys, "remove object", objects, names, 0);
//        Draw(imOrigin, vKeysOrigin, "origin",objects, names, 1);
//    }
//    return 0;
//}

//void LoadImages_robotcar(const string &strPathToSequence, vector<string> &vstrImageFilenames)
//{

//    string strPrefixLeft = strPathToSequence + "imagePath.txt";
//    ifstream pathFile(strPrefixLeft.c_str());
//    if(!pathFile.is_open()){
//        cerr << "Cannot open " << strPrefixLeft << endl;
//        exit(0);
//    }

//    while(!pathFile.eof())
//    {
//        string s;
//        getline(pathFile, s);
//        if(!s.empty()){
//            vstrImageFilenames.push_back(s);
//        }
//    }
//    pathFile.close();
//}

//void Draw(Mat im, vector<KeyPoint> keys, string title, vector<DetectedObject> objects, vector<string> name, int flag)
//{
//    cv::namedWindow(title);
//    const int n = keys.size();
//    const float r = 5;

//    for(int i=0;i<n;i++)
//    {
//        cv::Point2f pt1, pt2;
//        pt1.x=keys[i].pt.x-r;
//        pt1.y=keys[i].pt.y-r;
//        pt2.x=keys[i].pt.x+r;
//        pt2.y=keys[i].pt.y+r;

//        cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
//        cv::circle(im,keys[i].pt,2,cv::Scalar(0,255,0),-1);
//    }
//    for(int i = 0; i < objects.size(); i++)
//    {
//        DetectedObject& o = objects[i];
//        cv::rectangle(im, o.bounding_box, cv::Scalar(255,0,255), 2);

//        string class_name = name[o.object_class];

//        char str[255];

//        sprintf(str,"%s %%%.2f", class_name.c_str(), o.prob);
//        cv::putText(im, str, cv::Point2f(o.bounding_box.x,o.bounding_box.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,255), 1);
//    }
////   cv::Mat im_sized;
////   cv::Size cSize(im.cols/2, im.rows/2);
////   cv::resize(im, im_sized, cSize);
//   cv::imshow(title,im);
//   cv::waitKey(1);
//}



#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <chrono>

#include "stats.h" // Stats structure definition
#include "utils.h" // Drawing and printing functions

using namespace std;
using namespace cv;

const double akaze_thresh = 3e-4; // AKAZE detection threshold set to locate about 1000 keypoints
const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
const int bb_min_inliers = 100; // Minimal number of inliers to draw bounding box
const int stats_update_period = 10; // On-screen statistics are updated every 10 frames

class Tracker
{
public:
    Tracker(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher) :
        detector(_detector),
        matcher(_matcher)
    {}

    void setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats);
    Mat process(const Mat frame, Stats& stats);
    Ptr<Feature2D> getDetector() {
        return detector;
    }
protected:
    Ptr<Feature2D> detector;
    Ptr<DescriptorMatcher> matcher;
    Mat first_frame, first_desc;
    vector<KeyPoint> first_kp;
    vector<Point2f> object_bb;
};

void Tracker::setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats)
{
    first_frame = frame.clone();
    detector->detectAndCompute(first_frame, noArray(), first_kp, first_desc);
    stats.keypoints = (int)first_kp.size();
    drawBoundingBox(first_frame, bb);
    putText(first_frame, title, Point(0, 60), FONT_HERSHEY_PLAIN, 5, Scalar::all(0), 4);
    object_bb = bb;
}

Mat Tracker::process(const Mat frame, Stats& stats)
{
    vector<KeyPoint> kp;
    Mat desc;
    detector->detectAndCompute(frame, noArray(), kp, desc);
    stats.keypoints = (int)kp.size();

    vector< vector<DMatch> > matches;
    vector<KeyPoint> matched1, matched2;
    matcher->knnMatch(first_desc, desc, matches, 2);
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            matched1.push_back(first_kp[matches[i][0].queryIdx]);
            matched2.push_back(      kp[matches[i][0].trainIdx]);
        }
    }
    stats.matches = (int)matched1.size();

    Mat inlier_mask, homography;
    vector<KeyPoint> inliers1, inliers2;
    vector<DMatch> inlier_matches;
    if(matched1.size() >= 4) {
        homography = findHomography(Points(matched1), Points(matched2),
                                    RANSAC, ransac_thresh, inlier_mask);
    }

    if(matched1.size() < 4 || homography.empty()) {
        Mat res;
        hconcat(first_frame, frame, res);
        stats.inliers = 0;
        stats.ratio = 0;
        return res;
    }
    for(unsigned i = 0; i < matched1.size(); i++) {
        if(inlier_mask.at<uchar>(i)) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            inlier_matches.push_back(DMatch(new_i, new_i, 0));
        }
    }
    stats.inliers = (int)inliers1.size();
    stats.ratio = stats.inliers * 1.0 / stats.matches;

    vector<Point2f> new_bb;
    perspectiveTransform(object_bb, new_bb, homography);
    Mat frame_with_bb = frame.clone();
    if(stats.inliers >= bb_min_inliers) {
        drawBoundingBox(frame_with_bb, new_bb);
    }
    Mat res;
    drawMatches(first_frame, inliers1, frame_with_bb, inliers2,
                inlier_matches, res,
                Scalar(255, 0, 0), Scalar(255, 0, 0));
    return res;
}

int main(int argc, char **argv)
{
    if(argc < 4) {
        cerr << "Usage: " << endl <<
                "akaze_track input_path output_path bounding_box" << endl;
        return 1;
    }
    VideoCapture video_in(argv[1]);
    VideoWriter  video_out(argv[2],
                           (int)video_in.get(CAP_PROP_FOURCC),
                           (int)video_in.get(CAP_PROP_FPS),
                           Size(2 * (int)video_in.get(CAP_PROP_FRAME_WIDTH),
                                2 * (int)video_in.get(CAP_PROP_FRAME_HEIGHT)));

    if(!video_in.isOpened()) {
        cerr << "Couldn't open " << argv[1] << endl;
        return 1;
    }
    if(!video_out.isOpened()) {
        cerr << "Couldn't open " << argv[2] << endl;
        return 1;
    }

    vector<Point2f> bb;
    FileStorage fs(argv[3], FileStorage::READ);
    if(fs["bounding_box"].empty()) {
        cerr << "Couldn't read bounding_box from " << argv[3] << endl;
        return 1;
    }
    fs["bounding_box"] >> bb;

    Stats stats, akaze_stats, orb_stats;
    Ptr<AKAZE> akaze = AKAZE::create();
    akaze->setThreshold(akaze_thresh);
    Ptr<ORB> orb = ORB::create();
    orb->setMaxFeatures(stats.keypoints);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    Tracker akaze_tracker(akaze, matcher);
    Tracker orb_tracker(orb, matcher);

    Mat frame;
    video_in >> frame;
    akaze_tracker.setFirstFrame(frame, bb, "AKAZE", stats);
    orb_tracker.setFirstFrame(frame, bb, "ORB", stats);

    Stats akaze_draw_stats, orb_draw_stats;
    int frame_count = (int)video_in.get(CAP_PROP_FRAME_COUNT);
    Mat akaze_res, orb_res, res_frame;
    for(int i = 1; i < frame_count; i++) {
        bool update_stats = (i % stats_update_period == 0);
        video_in >> frame;

        akaze_res = akaze_tracker.process(frame, stats);
        akaze_stats += stats;
        if(update_stats) {
            akaze_draw_stats = stats;
        }

        orb->setMaxFeatures(stats.keypoints);
        orb_res = orb_tracker.process(frame, stats);
        orb_stats += stats;
        if(update_stats) {
            orb_draw_stats = stats;
        }

        drawStatistics(akaze_res, akaze_draw_stats);
        drawStatistics(orb_res, orb_draw_stats);
        vconcat(akaze_res, orb_res, res_frame);
        video_out << res_frame;
        cout << i << "/" << frame_count - 1 << endl;
    }
    akaze_stats /= frame_count - 1;
    orb_stats /= frame_count - 1;
    printStatistics("AKAZE", akaze_stats);
    printStatistics("ORB", orb_stats);
    return 0;
}
