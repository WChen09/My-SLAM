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

//darknet
//#include"Thirdparty/darknet/src/yolo.h"
//#include <string>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <fstream>
//#include <vector>
//#include <set>
//#include "ORBextractor.h"
//#include "Thirdparty/darknet/src/yolo.h"
//#include "Thirdparty/darknet/src/box.h"
//
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

//    Yolo yolo(argv[2]);
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
////        (*pORBextractorLeft)(imGray, cv::Mat(), vKeys, Descriptors, objects);
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

//int main( int /*argc*/, char** /*argv*/ )
//{
//    const int MAX_CLUSTERS = 5;
//    Scalar colorTab[] =
//    {
//        Scalar(0, 0, 255),
//        Scalar(0,255,0),
//        Scalar(255,100,100),
//        Scalar(255,0,255),
//        Scalar(0,255,255)
//    };

//    Mat img(500, 500, CV_8UC3);
//    RNG rng(12345);

//    for(;;)
//    {
//        int k, clusterCount = rng.uniform(2, MAX_CLUSTERS+1);
//        int i, sampleCount = rng.uniform(1, 1001);
//        Mat points(sampleCount, 1, CV_32FC2), labels;

//        clusterCount = MIN(clusterCount, sampleCount);
//        Mat centers;

//        /* generate random sample from multigaussian distribution */
//        for( k = 0; k < clusterCount; k++ )
//        {
//            Point center;
//            center.x = rng.uniform(0, img.cols);
//            center.y = rng.uniform(0, img.rows);
//            Mat pointChunk = points.rowRange(k*sampleCount/clusterCount,
//                                             k == clusterCount - 1 ? sampleCount :
//                                             (k+1)*sampleCount/clusterCount);
//            rng.fill(pointChunk, RNG::NORMAL, Scalar(center.x, center.y), Scalar(img.cols*0.05, img.rows*0.05));
//        }

//        randShuffle(points, 1, &rng);

//        kmeans(points, clusterCount, labels,
//            TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
//               3, KMEANS_PP_CENTERS, centers);

//        img = Scalar::all(0);

//        for( i = 0; i < sampleCount; i++ )
//        {
//            int clusterIdx = labels.at<int>(i);
//            Point ipt = points.at<Point2f>(i);
//            circle( img, ipt, 2, colorTab[clusterIdx], FILLED, LINE_AA );
//        }

//        imshow("clusters", img);

//        char key = (char)waitKey();
//        if( key == 27 || key == 'q' || key == 'Q' ) // 'ESC'
//            break;
//    }

//    std::vector<std::vector<float>> da(2);
//    da.at(0).push_back(1);
//    da.at(0).push_back(2);
//    da.at(1).push_back(2);
//    da.at(1).push_back(3);

//    std::vector<std::vector<float>> dd;
//    std::vector<float> cc;
//    cc.push_back(1.2);

//    dd.push_back(cc);
//    dd = da;
//    std::cout << std::endl;

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
//
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//int main (int argc, char** argv)
//{
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//
//  // Fill in the cloud data
//  pcl::PCDReader reader;
//  // Replace the path below with the path where you saved your file
//  reader.read<pcl::PointXYZ> ("./table_scene_lms400.pcd", *cloud);
//
//  std::cerr << "Cloud before filtering: " << std::endl;
//  std::cerr << *cloud << std::endl;
//
//  // Create the filtering object
//  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//  sor.setInputCloud (cloud);
//  sor.setMeanK (50);
//  sor.setStddevMulThresh (1.0);
//  sor.filter (*cloud_filtered);
//
//  std::cerr << "Cloud after filtering: " << std::endl;
//  std::cerr << *cloud_filtered << std::endl;
//
//  pcl::visualization::PCLVisualizer viewer("Cloud viewer");
//  viewer.setCameraPosition(0,0,-3.0,0,-1,0);
//  viewer.addCoordinateSystem(0.3);
//
//
//  viewer.addPointCloud(cloud_filtered);
//  while(!viewer.wasStopped())
//  viewer.spinOnce(100);
//
//  pcl::PCDWriter writer;
//  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
//
//  sor.setNegative (true);
//  sor.filter (*cloud_filtered);
//  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
//
//
//  cv::calcOpticalFlowFarneback();
//  cv::calcOpticalFlowPyrLK();
//
//
//  return 0;
//}


//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/videoio/videoio.hpp"
//#include "opencv2/highgui/highgui.hpp"

//#include <iostream>
//#include <ctype.h>

//using namespace cv;
//using namespace std;

//static void help()
//{
//    // print a welcome message, and the OpenCV version
//    cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
//            "Using OpenCV version " << CV_VERSION << endl;
//    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
//    cout << "\nHot keys: \n"
//            "\tESC - quit the program\n"
//            "\tr - auto-initialize tracking\n"
//            "\tc - delete all the points\n"
//            "\tn - switch the \"night\" mode on/off\n"
//            "To add/remove a feature point click it\n" << endl;
//}

//Point2f point;
//bool addRemovePt = false;

//static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
//{
//    if( event == EVENT_LBUTTONDOWN )
//    {
//        point = Point2f((float)x, (float)y);
//        addRemovePt = true;
//    }
//}

//int main( int argc, char** argv )
//{
//    VideoCapture cap;
//    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
//    Size subPixWinSize(10,10), winSize(31,31);

//    const int MAX_COUNT = 500;
//    bool needToInit = false;
//    bool nightMode = false;

//    cv::CommandLineParser parser(argc, argv, "{@input||}{help h||}");
//    string input = parser.get<string>("@input");
//    if (parser.has("help"))
//    {
//        help();
//        return 0;
//    }
//    if( input.empty() )
//        cap.open(0);
//    else if( input.size() == 1 && isdigit(input[0]) )
//        cap.open(input[0] - '0');
//    else
//        cap.open(input);

//    if( !cap.isOpened() )
//    {
//        cout << "Could not initialize capturing...\n";
//        return 0;
//    }

//    namedWindow( "LK Demo", 1 );
//    setMouseCallback( "LK Demo", onMouse, 0 );

//    Mat gray, prevGray, image, frame;
//    vector<Point2f> points[2];

//    for(;;)
//    {
//        cap >> frame;
//        if( frame.empty() )
//            break;

//        frame.copyTo(image);
//        cvtColor(image, gray, COLOR_BGR2GRAY);

//        if( nightMode )
//            image = Scalar::all(0);

//        if( needToInit )
//        {
//            // automatic initialization
//            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
//            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
//            addRemovePt = false;
//        }
//        else if( !points[0].empty() )
//        {
//            vector<uchar> status;
//            vector<float> err;
//            if(prevGray.empty())
//                gray.copyTo(prevGray);
//            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
//                                 3, termcrit, 0, 0.001);
//            size_t i, k;
//            for( i = k = 0; i < points[1].size(); i++ )
//            {
//                if( addRemovePt )
//                {
//                    if( norm(point - points[1][i]) <= 5 )
//                    {
//                        addRemovePt = false;
//                        continue;
//                    }
//                }

//                if( !status[i] )
//                    continue;

//                points[1][k++] = points[1][i];
//                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
//            }
//            points[1].resize(k);
//        }

//        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
//        {
//            vector<Point2f> tmp;
//            tmp.push_back(point);
//            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
//            points[1].push_back(tmp[0]);
//            addRemovePt = false;
//        }

//        needToInit = false;
//        imshow("LK Demo", image);

//        char c = (char)waitKey(10);
//        if( c == 27 )
//            break;
//        switch( c )
//        {
//        case 'r':
//            needToInit = true;
//            break;
//        case 'c':
//            points[0].clear();
//            points[1].clear();
//            break;
//        case 'n':
//            nightMode = !nightMode;
//            break;
//        }

//        std::swap(points[1], points[0]);
//        cv::swap(prevGray, gray);
//    }

//}


#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include <iostream>
using namespace cv;
using namespace std;



int main()
{
//     generate marker
//        Mat MarkerImage;
//        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

//        string savePath = "/home/wchen/kinect_image/marker/";

//        for(int i = 1; i < 50; i++ )
//        {
//            MarkerImage.release();
//            cv::aruco::drawMarker(dictionary, i, 300, MarkerImage, 1);
//            stringstream id_str;

//            id_str << i << ".jpg";
//            string id ;
//            id_str >> id;

//            string img_path = savePath + id;

//            cv::imshow("23Marker", MarkerImage);
//            waitKey(1000);
//            cv::imwrite(img_path, MarkerImage);

//        }

    cv::Mat cameraMatrix = (cv::Mat_<float>(3,3)<<
                              523.49,    0.0,         490.7171,
                              0.0,       522.9165,    272.2577,
                              0.0,       0.0,         1.   );

    cv::Mat distCoeffs = (cv::Mat_<float>(1,5) <<
                          0.0380, -0.0350, 2.9601e-05, -3.4270e-05, -0.0041);


    cv::Mat inputImage;

    string imagePath = "/home/wchen/kinect_image/marker_on_ground/";
    string savePath = "/home/wchen/kinect_image/marker_detection_result/";

//    string imagePath = "/home/wchen/kinect_image/single_marker/";
//    string savePath = "/home/wchen/kinect_image/single_marker_detection/";

    cv::aruco::DetectorParameters parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    for(int i = 0; i < 5; i++)
    {
        stringstream id_strs;
        id_strs << "000" << i << "_color.jpg";
        string id_str;
        id_strs >> id_str;

        string imagePath_i = imagePath + id_str;

        std::cout <<imagePath_i << endl;
        inputImage = cv::imread(imagePath_i);


        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

        cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

        for(size_t k = 0; k < markerIds.size(); k++)
        {
            std::cout << markerIds.at(k) << std::endl;
        }

        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.075, cameraMatrix, distCoeffs, rvecs, tvecs);

//        cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds, Scalar(255,0,0));
        for(size_t j = 0; j < tvecs.size(); j++)
        {
            cv::aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, rvecs.at(j), tvecs.at(j), 0.1);
        }

        cv::imshow("marker", inputImage);
        string imageSavePath = savePath + id_str;

        cv::imwrite(imageSavePath, inputImage);
        cv::waitKey(0);
    }

    return 0;
}






