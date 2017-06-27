#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core.hpp>
#include <vector>
#include "stats.h"
#include "Thirdparty/darknet/src/object.h"

using namespace std;
using namespace cv;

void drawBoundingBox(Mat image, vector<Point2f> bb);
void drawStatistics(Mat image, const Stats& stats);
void printStatistics(string name, Stats stats);
vector<Point2f> Points(vector<KeyPoint> keypoints);
void DrawDetector(cv::Mat& frame, std::vector<DetectedObject>& detectedBox, std::vector<int>& nTracker_);
void DrawDetector(cv::Mat& frame, std::vector<DetectedObject>& detectedBox, std::vector<int>& nTracker_,
                  std::vector<DetectedObject>& predictDetectedBox, std::vector<int>& predictNTracker_);
void DrawKpsWithinObject(cv::Mat& frame, std::vector<cv::KeyPoint> KPsIn, std::vector<KeyPoint> KPsOut);
void DrawKpsWithinObject(cv::Mat& frame, std::vector<std::vector<cv::KeyPoint>> KPsIn, std::vector<cv::KeyPoint> KPsOut);

void drawBoundingBox(Mat image, vector<Point2f> bb)
{
    for(unsigned i = 0; i < bb.size() - 1; i++) {
        line(image, bb[i], bb[i + 1], Scalar(0, 0, 255), 2);
    }
    line(image, bb[bb.size() - 1], bb[0], Scalar(0, 0, 255), 2);
}

void drawStatistics(Mat image, const Stats& stats)
{
    static const int font = FONT_HERSHEY_PLAIN;
    stringstream str1, str2, str3;

    str1 << "Matches: " << stats.matches;
    str2 << "Inliers: " << stats.inliers;
    str3 << "Inlier ratio: " << setprecision(2) << stats.ratio;

    putText(image, str1.str(), Point(0, image.rows - 90), font, 2, Scalar::all(255), 3);
    putText(image, str2.str(), Point(0, image.rows - 60), font, 2, Scalar::all(255), 3);
    putText(image, str3.str(), Point(0, image.rows - 30), font, 2, Scalar::all(255), 3);
}

void printStatistics(string name, Stats stats)
{
    cout << name << endl;
    cout << "----------" << endl;

    cout << "Matches " << stats.matches << endl;
    cout << "Inliers " << stats.inliers << endl;
    cout << "Inlier ratio " << setprecision(2) << stats.ratio << endl;
    cout << "Keypoints " << stats.keypoints << endl;
    cout << endl;
}

vector<Point2f> Points(vector<KeyPoint> keypoints)
{
    vector<Point2f> res;
    for(unsigned i = 0; i < keypoints.size(); i++) {
        res.push_back(keypoints[i].pt);
    }
    return res;
}


void DrawDetector(cv::Mat& frame, std::vector<DetectedObject>& detectedBox, std::vector<int>& nTracker_)
{
    for(size_t i = 0; i < detectedBox.size(); i++ )
    {
        cv::Rect currentRect(detectedBox[i].bounding_box);
        cv::rectangle(frame, currentRect, cv::Scalar(255, 0, 0), 1, CV_AA);
        char str[255];
        sprintf(str,"%d", nTracker_[i]);
        cv::putText(frame, str, cv::Point2f(currentRect.x, currentRect.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
    }
}

void DrawDetector(cv::Mat& frame, std::vector<DetectedObject>& detectedBox, std::vector<int>& nTracker_,
                  std::vector<DetectedObject>& predictDetectedBox, std::vector<int>& predictNTracker_)
{
    for(size_t i = 0; i < detectedBox.size(); i++ )
    {
        cv::Rect currentRect(detectedBox[i].bounding_box);
        cv::rectangle(frame, currentRect, cv::Scalar(255, 0, 0), 1, CV_AA);
        char str[255];
        sprintf(str,"%d", nTracker_[i]);
        cv::putText(frame, str, cv::Point2f(currentRect.x, currentRect.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1);
    }

    for(size_t i = 0; i < predictDetectedBox.size(); i++ )
    {
        cv::Rect currentRect(predictDetectedBox[i].bounding_box);
        cv::rectangle(frame, currentRect, cv::Scalar(0, 255, 0), 1, CV_AA);
        char str[255];
        sprintf(str,"%d", predictNTracker_[i]);
        cv::putText(frame, str, cv::Point2f(currentRect.x, currentRect.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1);
    }
}

void DrawKpsWithinObject(cv::Mat& frame, std::vector<cv::KeyPoint> KPsIn, std::vector<cv::KeyPoint> KPsOut)
{
    const float r = 5;

    for(size_t i = 0; i < KPsIn.size(); i++)
    {
        cv::Point2f pt1,pt2;
        pt1.x=KPsIn[i].pt.x-r;
        pt1.y=KPsIn[i].pt.y-r;
        pt2.x=KPsIn[i].pt.x+r;
        pt2.y=KPsIn[i].pt.y+r;

        cv::rectangle(frame,pt1,pt2,cv::Scalar(255,0,0));//magenta color
        cv::circle(frame,KPsIn[i].pt,2,cv::Scalar(255,0,0),-1);

    }


    for(size_t i = 0; i < KPsOut.size(); i++)
    {
        cv::Point2f pt1,pt2;
        pt1.x=KPsOut[i].pt.x-r;
        pt1.y=KPsOut[i].pt.y-r;
        pt2.x=KPsOut[i].pt.x+r;
        pt2.y=KPsOut[i].pt.y+r;

        cv::rectangle(frame,pt1,pt2,cv::Scalar(0,255,0));//magenta color
        cv::circle(frame,KPsOut[i].pt,2,cv::Scalar(0,255,0),-1);

    }
}

void DrawKpsWithinObject(cv::Mat& frame, std::vector<std::vector<cv::KeyPoint>> KPsIn, std::vector<cv::KeyPoint> KPsOut)
{
    const float r = 5;

    for(size_t iobject = 0; iobject < KPsIn.size(); iobject++)
    {
        for(size_t i = 0; i < KPsIn[iobject].size(); i++)
        {
            cv::Point2f pt1,pt2;
            pt1.x=KPsIn[iobject][i].pt.x-r;
            pt1.y=KPsIn[iobject][i].pt.y-r;
            pt2.x=KPsIn[iobject][i].pt.x+r;
            pt2.y=KPsIn[iobject][i].pt.y+r;

            cv::rectangle(frame,pt1,pt2,cv::Scalar(255,0,0));//magenta color
            cv::circle(frame,KPsIn[iobject][i].pt,2,cv::Scalar(255,0,0),-1);
        }
    }


    for(size_t i = 0; i < KPsOut.size(); i++)
    {
        if(KPsOut.at(i).class_id != -1)
            continue;
        cv::Point2f pt1,pt2;
        pt1.x=KPsOut[i].pt.x-r;
        pt1.y=KPsOut[i].pt.y-r;
        pt2.x=KPsOut[i].pt.x+r;
        pt2.y=KPsOut[i].pt.y+r;

        cv::rectangle(frame,pt1,pt2,cv::Scalar(0,255,0));//magenta color
        cv::circle(frame,KPsOut[i].pt,2,cv::Scalar(0,255,0),-1);

    }
}

#endif // UTILS_H
