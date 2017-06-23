#include "ObjectTracker.h"
#include <thread>
#include <chrono>

// ORB Extractor parameters
const int nFeatures = 800;
const float scaleFactor = 1.2;
const int nLevels = 8;
const int iniFAST = 20;
const int minThFAST = 2;

ObjectTracker::ObjectTracker(const float maxdistTh, const Size frameSize_): dist_thres(maxdistTh), frameSize(frameSize_)
{
    initObject = new DetectedObject(-1, -1, cv::Rect(0, 0, 0, 0));
    size_t MaxObjects = 10;

    mvlastDetecedBox.resize(MaxObjects);
    mvnLastTrackObjectID.resize(MaxObjects);
    for(size_t i = 0; i < mvlastDetecedBox.size(); i++)
    {
        mvlastDetecedBox[i] = *initObject;
        mvnLastTrackObjectID[i] = -1;
    }

    frameId = 0;
    trackId = 0;

    vframeObjectWithIdpair = new std::vector<std::pair<vObjects, std::vector<int>>>();

    imgStorePath = "./result.avi";

    extractorIn = new ORB_SLAM2::ORBextractor(nFeatures, scaleFactor, nLevels, iniFAST, minThFAST);
    extractorOut = new ORB_SLAM2::ORBextractor(nFeatures*2, scaleFactor, nLevels, iniFAST, minThFAST);

    matcher = DescriptorMatcher::create("BruteForce-Hamming");

    vframeInObjectORBpair = new std::vector<std::pair<std::vector<std::vector<cv::KeyPoint>>, std::vector<cv::Mat>>>();
    vframeOutObjectORBpair = new std::vector<std::pair<std::vector<cv::KeyPoint>, cv::Mat>>();

    writeFrame = new cv::VideoWriter();

    if (!writeFrame->isOpened())
    {
        writeFrame->open(imgStorePath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, frameSize, true);
    }

}

ObjectTracker::~ObjectTracker()
{

    delete APS;
    delete extractorIn;
    delete extractorOut;
    delete matcher;
    delete vframeObjectWithIdpair;
    delete writeFrame;
    delete assignment;
    delete vDistance;
    delete vframeInObjectORBpair;
    delete vframeOutObjectORBpair;

}
void ObjectTracker::ExtractORB(int flag, const Mat &im, const std::vector<DetectedObject> vCurrentObjects)
{
    if(flag == 0)
        (*extractorOut)(im, cv::Mat(), kpsOut, descriptorsOut, vCurrentObjects, 0);
    else
        (*extractorIn)(im, cv::Mat(), kpsIn, descriptorsIn, vCurrentObjects, 1);
}

void ObjectTracker::grabImgWithObjects(Mat &frame, vObjects &vCurrentObjects)
{
    vnCurrentTrackObjectID = new std::vector<int>(vCurrentObjects.size(), -1);

    cv::Mat greyFrame;
    cvtColor(frame, greyFrame, CV_RGB2GRAY);


//    extractor->extracteORBInObject(greyFrame, cv::Mat(), kpsIn, descriptorsIn, vCurrentObjects);
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::thread inThread(&ObjectTracker::ExtractORB, this, 1, greyFrame, vCurrentObjects);
    std::thread outThread(&ObjectTracker::ExtractORB, this, 0, greyFrame, vCurrentObjects);
    outThread.join();
    inThread.join();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::cout << "AllBox " << std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count() << " seconds." << std::endl;
//    DrawKpsWithinObject(frame, kpsIn, kpsOut);

    reorganizeORB(kpsIn, descriptorsIn, vCurrentObjects);

    DrawKpsWithinObject(frame, *vkpsInObject, kpsOut);

    if(frameId == 0)
    {

        for(size_t i = 0; i < vCurrentObjects.size(); i++)
        {
            vnCurrentTrackObjectID->at(i) = trackId;
            mvlastDetecedBox[i] = vCurrentObjects.at(i);
            mvnLastTrackObjectID[i] = vnCurrentTrackObjectID->at(i);
            trackId++;
        }

        // store current frame information
        vframeObjectWithIdpair->push_back(std::make_pair(mvlastDetecedBox,
                                                         mvnLastTrackObjectID));
        vframeInObjectORBpair->push_back(std::make_pair(*vkpsInObject, *vdescriptorsInObject));
        vframeOutObjectORBpair->push_back(std::make_pair(kpsOut,descriptorsOut));

        DrawDetector(frame, vCurrentObjects, *vnCurrentTrackObjectID);
        lastFrame = frame.clone();

        if (writeFrame->isOpened())
            *writeFrame << frame;

        cv::imshow("Video", frame);
        cv::waitKey(1);

        frameId++;

        delete vnCurrentTrackObjectID;
        delete vkpsInObject;
        delete vdescriptorsInObject;
        return;
    }

// track last frame
 // 1. hungarianAlgorithm assignment
    size_t N = mvnLastTrackObjectID.size() - std::count(mvnLastTrackObjectID.begin(), mvnLastTrackObjectID.end(), -1);
    size_t M = vCurrentObjects.size();
    {
        vDistance = new std::vector<float>(N*M) ;
        assignment = new std::vector<int>(N,-1);

        float maxDist = 0;

        for(size_t i = 0; i < N; i++)
        {
            for(size_t j = 0; j < M; j++)
            {
                float dist =  CalcDistJaccard(vCurrentObjects.at(j).bounding_box, mvlastDetecedBox[i].bounding_box);
                vDistance->at(i + j * N) = dist;
                if(dist > maxDist)
                    maxDist = dist;
            }
        }

        APS->Solve(*vDistance, N, M, *assignment, AssignmentProblemSolver::optimal);

        // -----------------------------------
        // clean assignment from pairs with large distance
        // -----------------------------------
        for (size_t i = 0; i < assignment->size(); i++)
        {
            if (assignment->at(i) != -1)
            {
                if (vDistance->at(i + assignment->at(i) * N) > dist_thres)
                {
                    assignment->at(i) = -1;
                }
            }
        }

        //        std::vector<int> assignment = CalculateAssignment(lastDetecedBox, currentDetectedBox);
        for(size_t i = 0; i < assignment->size(); i++)
        {
            if(assignment->at(i)!=-1)
                vnCurrentTrackObjectID->at(assignment->at(i)) = mvnLastTrackObjectID[i];
        }

 // 2. using ORB Matcher retrack
        for(size_t i = 0; i < assignment->size(); i++)
        {
            if(assignment->at(i) != -1)
                continue;

        }




        delete vDistance;
        delete assignment;
    }

    // new track, add new track num
    for(size_t icurrentTrack = 0; icurrentTrack < vCurrentObjects.size(); icurrentTrack++)
    {
        if(vnCurrentTrackObjectID->at(icurrentTrack) == -1)
        {
            vnCurrentTrackObjectID->at(icurrentTrack) = trackId;
            trackId++;
        }
    }

    // store current frame to last frame
    for(size_t i = 0; i < mvnLastTrackObjectID.size(); i++)
    {
        if(i < M)
        {
            mvlastDetecedBox[i] = vCurrentObjects.at(i);
            mvnLastTrackObjectID[i] = vnCurrentTrackObjectID->at(i);
        }
        else
        {
            mvlastDetecedBox[i] = *initObject;
            mvnLastTrackObjectID[i] = -1;
        }

    }

    DrawDetector(frame, vCurrentObjects, *vnCurrentTrackObjectID);


    // store current frame information
    vframeObjectWithIdpair->push_back(std::make_pair(mvlastDetecedBox,
                                                     mvnLastTrackObjectID));
    vframeInObjectORBpair->push_back(std::make_pair(*vkpsInObject, *vdescriptorsInObject));
    vframeOutObjectORBpair->push_back(std::make_pair(kpsOut,descriptorsOut));
    lastFrame = frame.clone();

    if (writeFrame->isOpened())
        *writeFrame << frame;

    cv::imshow("Video", frame);
    cv::waitKey(1);

    delete vnCurrentTrackObjectID;

    frameId++;

}

double ObjectTracker::CalcDistJaccard(Rect &current, Rect &former)
{

    float intArea = (current & former).area();
    float unionArea = current.area() + former.area() - intArea;

    return 1 - intArea / unionArea;
}

std::vector<int> ObjectTracker::CalculateAssignment(vObjects &vFormerObjects, vObjects &vCurrentObjects)
{

    size_t N = vFormerObjects.size();
    size_t M = vCurrentObjects.size();

    std::vector<int> assignment(N, -1); // Appointments

    std::vector<float> vDistance(N*M);
    float maxDist = 0;

    for(size_t i = 0; i < N; i++)
    {
        for(size_t j = 0; j < M; j++)
        {
            float dist =  CalcDistJaccard(vCurrentObjects[j].bounding_box, vFormerObjects[i].bounding_box);
            vDistance[i + j * N] = dist;
            if(dist > maxDist)
                maxDist = dist;
        }
    }

    AssignmentProblemSolver APS;
    APS.Solve(vDistance, N, M, assignment, AssignmentProblemSolver::optimal);
    return assignment;
}

void ObjectTracker::CutInBiggestBox(vObjects &vObjects_, Rect &FrameBox)
{
    for(size_t iObject = 0; iObject < vObjects_.size(); iObject++)
    {
        DetectedObject object = vObjects_[iObject];
        cv::Rect inBox = object.bounding_box & FrameBox;
        float areaRatio = (float)inBox.area() / (float)object.bounding_box.area();
        if(areaRatio < 1)
        {
            int rigthDownPointx = object.bounding_box.x + object.bounding_box.width;
            int rightDownPointy = object.bounding_box.y + object.bounding_box.height;
            if(rigthDownPointx > FrameBox.width)
                vObjects_[iObject].bounding_box.width = FrameBox.width - rigthDownPointx;
            if(rightDownPointy > FrameBox.height)
                vObjects_[iObject].bounding_box.height = FrameBox.height - rightDownPointy;
        }
    }
}

void ObjectTracker::reorganizeORB(std::vector<KeyPoint> KpsIn, Mat descriptorsIn, std::vector<DetectedObject> ObjectBox)
{
//    std::cout << "whole: " << descriptorsIn << std::endl << std::endl << std::endl;
    std::vector<int> InObject(KpsIn.size(), -1);
    vkpsInObject = new std::vector<std::vector<cv::KeyPoint>>(ObjectBox.size());
    vdescriptorsInObject = new std::vector<cv::Mat>(ObjectBox.size());

    for(size_t iObject=0; iObject < ObjectBox.size(); iObject++)
    {
        // keyPoints
        std::vector<KeyPoint>& kpsInObject = vkpsInObject->at(iObject);

        DetectedObject currentObject = ObjectBox[iObject];
        cv::Rect box = currentObject.bounding_box;
        size_t nKPInobject = 0;

        for(size_t iKps = 0; iKps < KpsIn.size(); iKps++)
        {
            if(InObject[iKps] != -1)
                continue;

            cv::Point p(KpsIn[iKps].pt);
            if(box.contains(p))
            {
                kpsInObject.push_back(KpsIn[iKps]);
                InObject[iKps] = (int)iObject;
                nKPInobject++;
            }
            else
                continue;
        }
        // descriptors
        cv::Mat objectDescriptor = Mat::zeros((int)nKPInobject, 32, CV_8UC1);
        int nidescriptor = 0;
        for(size_t i=0; i<InObject.size(); i++)
        {
            if(InObject[i] != (int)iObject)
                continue;

//            std::cout << i << " " << descriptorsIn.row(i) << std::endl << std::endl;

            descriptorsIn.row(i).copyTo(objectDescriptor.row(nidescriptor));
            nidescriptor++;
        }
//        std::cout << objectDescriptor << std::endl;
        vdescriptorsInObject->at(iObject) = objectDescriptor;
    }

}
