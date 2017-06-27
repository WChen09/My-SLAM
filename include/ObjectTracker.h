#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include "stats.h" // Stats structure definition
#include "utils.h" // Drawing and printing functions
#include "Thirdparty/darknet/src/object.h"
#include "HungarianAlg.h"
#include "ORBextractor.h"

#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;

typedef std::vector<DetectedObject> vObjects;

class ObjectTracker
{
public:
    ObjectTracker(const float maxdistTh, const cv::Size frameSize_);
    ~ObjectTracker();

    void grabImgWithObjects(cv::Mat& frame, vObjects& vCurrentObjects);
    double CalcDistJaccard(cv::Rect& current, cv::Rect& former);

    std::vector<int> CalculateAssignment(vObjects& vFormerObjects,
                                         vObjects& vCurrentObjects);

    void CutInBiggestBox(vObjects& vObjects_, cv::Rect& FrameBox);

protected:

    std::vector<cv::KeyPoint> *kpsIn;
    cv::Mat *descriptorsIn;
    std::vector<cv::KeyPoint> *kpsOut;
    cv::Mat  *descriptorsOut;

    std::vector<std::vector<cv::KeyPoint>>* vkpsInObject;
    std::vector<cv::Mat>* vdescriptorsInObject;

    void ExtractORB(int flag, const cv::Mat &im,
                    const std::vector<DetectedObject> vCurrentObjects);

    void reorganizeORB(std::vector<KeyPoint> KpsIn, cv::Mat descriptorsIn, std::vector<DetectedObject> ObjectBox);

    float Objectmatcher(cv::Mat lastDescriptor, cv::Mat currentDescrptor,
                         std::vector<cv::KeyPoint> vLastKps,
                         std::vector<cv::KeyPoint> vCurrentKps,
                         std::vector<cv::KeyPoint>& match1,
                         std::vector<cv::KeyPoint>& match2, string step, size_t &nInliers);

    //config parameters
    const float maxDistThres;
    const float minDistThres;
    const cv::Size frameSize;
    long unsigned int frameId;
    long unsigned int trackId;
    const double  nn_match_ratio;
    const double ransac_thresh; // RANSAC inlier threshold
    const float minInliersTh;

    ORB_SLAM2::ORBextractor* extractorIn;
    ORB_SLAM2::ORBextractor* extractorOut;

    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Record overall Box information
    std::vector<std::pair<std::vector<std::vector<cv::KeyPoint>>, std::vector<cv::Mat>>>* vframeInObjectORBpair;
    std::vector<std::pair<std::vector<cv::KeyPoint>, cv::Mat>>* vframeOutObjectORBpair;

    // Record descriptors and kps for every objects, no more than 10 objects
    std::vector<std::pair<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>>* mvObjectBag; // ith object with Kps and Descriptors
    std::vector<int>* mvObjectObservedTimes; // record the times of observing ith object
    std::vector<int>* mvObjectUnobservedTimes; //record the times of unobserving ith object

    // Record last frame's box information
    vObjects mvlastDetecedBox;
    std::vector<int> mvnLastTrackObjectID;
    cv::Mat lastFrame;

    std::vector<int>* vnCurrentTrackObjectID;

    AssignmentProblemSolver* APS;
    std::vector<float>* vDistance;
    std::vector<int>* assignment;

    string imgStorePath;

    DetectedObject* initObject;

    cv::VideoWriter* writeFrame;


};

#endif
