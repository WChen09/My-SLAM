#include "ObjectTracker.h"
#include <thread>
#include <chrono>
#include <nms.h>
// ORB Extractor parameters
const int nFeatures = 2000;
const float scaleFactor = 1.2;
const int nLevels = 8;
const int iniFAST = 20;
const int minThFAST = 2;

ObjectTracker::ObjectTracker(const float maxdistTh, const Size frameSize_): maxDistThres(maxdistTh), minDistThres(0.5), frameSize(frameSize_),
    nn_match_ratio(0.9f), ransac_thresh(2.5f), minInliersTh(0.3)
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

    imgStorePath = "./result.avi";

    matcher = DescriptorMatcher::create("BruteForce-Hamming");

    vframeInObjectORBpair = new std::vector<std::pair<std::vector<std::vector<cv::KeyPoint>>, std::vector<cv::Mat>>>();
    vframeOutObjectORBpair = new std::vector<std::pair<std::vector<cv::KeyPoint>, cv::Mat>>();

    writeFrame = new cv::VideoWriter();

    if (!writeFrame->isOpened())
    {
        writeFrame->open(imgStorePath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, frameSize, true);
    }

    mvObjectBag = new std::vector<std::pair<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>>(10);
    for(size_t i = 0; i < mvObjectBag->size(); i++)
    {
        mvObjectBag->at(i).first = -1;
    }
    mvObjectObservedTimes = new std::vector<int>(10, -1);
    mvObjectUnobservedTimes = new std::vector<int>(10, -1);
}

ObjectTracker::~ObjectTracker()
{

    delete APS;
    delete extractorIn;
    delete extractorOut;
    delete matcher;
    delete initObject;
    delete writeFrame;
    delete assignment;
    delete vDistance;
    delete vframeInObjectORBpair;
    delete vframeOutObjectORBpair;

    delete vkpsInObject;
    delete vdescriptorsInObject;

    delete kpsIn;
    delete kpsOut;
    delete descriptorsIn;
    delete descriptorsOut;

    delete mvObjectBag;
    delete mvObjectObservedTimes;
    delete mvObjectUnobservedTimes;
}
void ObjectTracker::ExtractORB(int flag, const Mat &im, const std::vector<DetectedObject> vCurrentObjects)
{
    float sumArea = 0;
    for(size_t i = 0; i < vCurrentObjects.size(); i++)
    {
        sumArea += vCurrentObjects.at(i).bounding_box.area();
    }
    float areaRatio = sumArea/(float)(im.cols * im.rows);
    int nFeaturesIn = cvRound(nFeatures * areaRatio);
    int nFeaturesOut = nFeatures - nFeaturesIn;

    if(flag == 0)
    {
        extractorOut = new ORB_SLAM2::ORBextractor(nFeaturesOut, scaleFactor, nLevels, iniFAST, minThFAST);
        kpsOut = new std::vector<cv::KeyPoint>();
        descriptorsOut = new cv::Mat();
        (*extractorOut)(im, cv::Mat(), *kpsOut, *descriptorsOut, vCurrentObjects, 0);
        delete extractorOut;
    }
    else
    {
        extractorIn = new ORB_SLAM2::ORBextractor(nFeaturesIn, scaleFactor, nLevels, iniFAST, minThFAST);
        kpsIn = new std::vector<cv::KeyPoint>();
        descriptorsIn = new cv::Mat();
        (*extractorIn)(im, cv::Mat(), *kpsIn, *descriptorsIn, vCurrentObjects, 1);
        delete extractorIn;
    }

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

    reorganizeORB(*kpsIn, *descriptorsIn, vCurrentObjects);

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
        vframeInObjectORBpair->push_back(std::make_pair(*vkpsInObject, *vdescriptorsInObject));
        vframeOutObjectORBpair->push_back(std::make_pair(*kpsOut, *descriptorsOut));

        // store object bags
        for(size_t i = 0; i < vCurrentObjects.size(); i++)
        {
            auto ObjectORBInfo = std::make_pair(vkpsInObject->at(i), vdescriptorsInObject->at(i));
            auto ObjectORBInfoWithId = std::make_pair(vnCurrentTrackObjectID->at(i), ObjectORBInfo);
            mvObjectBag->at(i) = ObjectORBInfoWithId;
            mvObjectObservedTimes->at(i) = 1;
            mvObjectUnobservedTimes->at(i) = 0;
        }

        DrawKpsWithinObject(frame, *vkpsInObject, *kpsOut);
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

        delete kpsIn;
        delete kpsOut;
        delete descriptorsIn;
        delete descriptorsOut;
        return;
    }

    // -----------------------------------
    // track
    // -----------------------------------
    // 1. hungarianAlgorithm assignment -- get initial correspondence
    // -----------------------------------

    size_t N = mvnLastTrackObjectID.size() - std::count(mvnLastTrackObjectID.begin(), mvnLastTrackObjectID.end(), -1);
    size_t M = vCurrentObjects.size();
    {
        vDistance = new std::vector<float>(N*M);
        assignment = new std::vector<int>(N,-1);

        for(size_t i = 0; i < N; i++)
        {
            for(size_t j = 0; j < M; j++)
            {
                float dist =  CalcDistJaccard(vCurrentObjects.at(j).bounding_box, mvlastDetecedBox[i].bounding_box);
                vDistance->at(i + j * N) = dist;
            }
        }

        APS->Solve(*vDistance, N, M, *assignment, AssignmentProblemSolver::optimal);

        // -----------------------------------
        // 2. refine correspondence
        // -----------------------------------
        // 2.1 clean assignment from pairs with large distance
        // -----------------------------------

        for (size_t i = 0; i < assignment->size(); i++)
        {
            if (assignment->at(i) != -1)
            {
                if (vDistance->at(i + assignment->at(i) * N) > maxDistThres)
                {
                    assignment->at(i) = -1;
                }
            }
        }

        // -----------------------------------
        // 2.2. use ORB Matcher recheck, if ratio < minInliersTh, clean such assignment
        // -----------------------------------

        std::cout << std::endl << "2.2 ORB matcher recheck: " << std::endl;
        std::pair<std::vector<std::vector<cv::KeyPoint>>, std::vector<cv::Mat>> lastKP_Des = vframeInObjectORBpair->at(vframeInObjectORBpair->size()-1);
        std::vector<std::vector<cv::KeyPoint>> vKPsObjects = lastKP_Des.first;
        std::vector<cv::Mat> vdescriptorsObjects = lastKP_Des.second;

        for(size_t i = 0; i < assignment->size(); i++)
        {
            if(assignment->at(i) == -1)
                continue;
            std::vector<cv::KeyPoint> lastiObjectKPs = vKPsObjects.at(i);
            cv::Mat lastiObjectDescriptors = vdescriptorsObjects.at(i);

            std::vector<cv::KeyPoint> currentiObjectKPs = vkpsInObject->at(assignment->at(i));
            cv::Mat currentiObjectDescriptors = vdescriptorsInObject->at(assignment->at(i));
            vector<KeyPoint> matched1, matched2;
            size_t nInliers=0;
            float ratio = Objectmatcher(lastiObjectDescriptors, currentiObjectDescriptors, lastiObjectKPs, currentiObjectKPs, matched1, matched2, "2.2.0", nInliers);
            if(nInliers > 20)
                continue;
            if(ratio <= minInliersTh * 0.8)
                assignment->at(i) = -1;
        }

        for(size_t i = 0; i < assignment->size(); i++)
        {
            if(assignment->at(i)!=-1)
                vnCurrentTrackObjectID->at(assignment->at(i)) = mvnLastTrackObjectID[i];
        }


        // -----------------------------------
        // 2.3. use ORB Matcher refind, if nmatches < 20, continue:
        // 2.3.1 firstly find in the current unassigned objects;
        // 2.3.2 finally, find match in the bag of objects
        // 2.3.3 if not find, then using last object box(with position in frame) to construct the current KPs vector, then use ORB matcher:
        // 2.3.4 nms
        // -----------------------------------

//         2.3.1 the current unassigned objects
        std::cout << "2.3.1 " << std::endl;
        for(size_t i = 0; i < assignment->size(); i++)
        {
            if(assignment->at(i) != -1)
                continue;

            std::vector<cv::KeyPoint>  ivlastKPsObjects = vKPsObjects.at(i);
            cv::Mat ilastDescriptors = vdescriptorsObjects.at(i);


            for(size_t icurrentObject = 0; icurrentObject < vkpsInObject->size(); icurrentObject++)
            {
                if(vnCurrentTrackObjectID->at(icurrentObject) != -1)
                    continue;

                std::vector<cv::KeyPoint> ivcurrentKPsObjects = vkpsInObject->at(icurrentObject);
                cv::Mat currentObjectDescriptors = vdescriptorsInObject->at(icurrentObject);

                vector<KeyPoint> matched1, matched2;
                size_t nInliers=0;
                float ratio = Objectmatcher(ilastDescriptors, currentObjectDescriptors, ivlastKPsObjects, ivcurrentKPsObjects, matched1, matched2, "2.3.1", nInliers);
                if( ratio <= minInliersTh*0.8)
                    continue;
                else
                {
                    assignment->at(i) = (int)icurrentObject;
                    vnCurrentTrackObjectID->at(icurrentObject) = mvnLastTrackObjectID[i];
                    break;
                }
            }
        }
        // 2.3.2 track in the bag of objects
        std::cout << "2.3.2 " << std::endl;
        std::vector<bool> hasBeenMatched(mvObjectBag->size(), false);
//        no more than 1 object with same id in current frame
        for(size_t i = 0; i < vnCurrentTrackObjectID->size(); i++)
        {
            if(vnCurrentTrackObjectID->at(i)==-1)
                continue;
            for(size_t j = 0; j < mvObjectBag->size(); j++)
            {
                if(mvObjectObservedTimes->at(j) <=0 )
                    continue;
                if(mvObjectBag->at(j).first == vnCurrentTrackObjectID->at(i))
                {
                    hasBeenMatched.at(j) = true;
                    continue;
                }
            }
        }

        for(size_t i = 0; i < vnCurrentTrackObjectID->size(); i++)
        {
            if(vnCurrentTrackObjectID->at(i) != -1)
                continue;
            for(size_t j = 0; j < mvObjectBag->size(); j++)
            {
                if(hasBeenMatched.at(j) || mvObjectObservedTimes->at(j) == -1)
                    continue;
                std::pair<std::vector<cv::KeyPoint>, cv::Mat> iObjectORB = mvObjectBag->at(j).second;
                std::vector<cv::KeyPoint> iObjectKps = iObjectORB.first;
                cv::Mat iObjectDes = iObjectORB.second;
                int ObjectId = mvObjectBag->at(j).first;

                vector<KeyPoint> matched1, matched2;
                size_t nInliers=0;
                float ratio = Objectmatcher(iObjectDes, vdescriptorsInObject->at(i), iObjectKps, vkpsInObject->at(i), matched1, matched2, "2.3.2", nInliers);
                if( ratio <= minInliersTh*0.8)
                    continue;
                else
                {
                    vnCurrentTrackObjectID->at(i) = ObjectId;
                    hasBeenMatched.at(j) = true;
                    break;
                }
            }
        }

        // 2.3.3 if not find in the current objects, then find the zone near the last box
        std::cout << "2.3.3 " << std::endl;
        for(size_t i = 0; i < assignment->size(); i++)
        {
            if(assignment->at(i) != -1)
                continue;

            std::vector<cv::KeyPoint>  ivlastKPsObjects = vKPsObjects.at(i);
            cv::Mat ilastDescriptors = vdescriptorsObjects.at(i);

            DetectedObject lastObjectNoMatched = mvlastDetecedBox[i];
            cv::Rect lastBox = lastObjectNoMatched.bounding_box;
            int classId = lastObjectNoMatched.object_class;
            std::vector<int> findKPsId;
            std::vector<cv::KeyPoint> ivcurrentKPsObjects;

            for(int iKpsOut = 0; iKpsOut < kpsOut->size(); iKpsOut++)
            {
                if(!lastBox.contains(kpsOut->at(iKpsOut).pt) || kpsOut->at(iKpsOut).class_id != -1)
                    continue;
                else
                {
                    findKPsId.push_back(iKpsOut);
                    kpsOut->at(iKpsOut).class_id = classId;
                    ivcurrentKPsObjects.push_back(kpsOut->at(iKpsOut));
                }
            }
            size_t outSize = findKPsId.size();
            for(size_t i = 0; i < kpsIn->size(); i++)
            {
                if(!lastBox.contains(kpsIn->at(i).pt))
                    continue;
                else
                {
                    findKPsId.push_back(i);
                    kpsIn->at(i).class_id = classId;
                    ivcurrentKPsObjects.push_back(kpsIn->at(i));
                }
            }

            cv::Mat currentObjectDescriptor = cv::Mat::zeros((int)findKPsId.size(), 32, CV_8UC1);

            for(size_t i = 0; i < findKPsId.size(); i++)
            {
                if(i < outSize)
                    descriptorsOut->row(findKPsId.at(i)).copyTo(currentObjectDescriptor.row(i));
                else
                    descriptorsIn->row(findKPsId.at(i)).copyTo(currentObjectDescriptor.row(i));
            }

            vector<KeyPoint> matched1, matched2;
            size_t nInliers=0;
            float ratio = Objectmatcher(ilastDescriptors, currentObjectDescriptor, ivlastKPsObjects, ivcurrentKPsObjects, matched1, matched2, "2.3.3", nInliers);
            if( ratio <= minInliersTh*0.8)
                continue;
            else
            {
                vCurrentObjects.push_back(lastObjectNoMatched);
                vkpsInObject->push_back(ivcurrentKPsObjects);
                vdescriptorsInObject->push_back(currentObjectDescriptor);
                vnCurrentTrackObjectID->push_back(mvnLastTrackObjectID[i]);
                assignment->at(i) = (int)vkpsInObject->size();
                break;
            }
        }
        delete vDistance;
        delete assignment;
    }

    // 2.3.4 nms - remove the wrong adding in 2.3.3
    std::vector<cv::Rect> objectRects, resRects;
    std::vector<float> scores;
    float overlopTh = 0.7;
    for(size_t i = 0; i < vCurrentObjects.size(); i++)
    {
        objectRects.push_back(vCurrentObjects.at(i).bounding_box);
        scores.push_back(vCurrentObjects.at(i).prob);
    }
    nms2(objectRects, scores, resRects, overlopTh);
    std::vector<size_t> idx;
    if(resRects.size() < vCurrentObjects.size())
    {
        std::vector<bool> bfind(vCurrentObjects.size(), false);
        for(size_t i_ = 0; i_ < vCurrentObjects.size(); i_++)
        {
            for(size_t j = 0; j < resRects.size(); j++)
            {
                if(CalcDistJaccard(vCurrentObjects.at(i_).bounding_box, resRects.at(j)) == 0)
                {
                    bfind.at(i_) = true;
                    break;
                }
            }
            if(!bfind.at(i_))
                idx.push_back(i_);
        }
        // exit bug
        for(size_t k = 0; k < idx.size(); k++)
        {
            vCurrentObjects.erase(vCurrentObjects.begin()+idx.at(k));
            vkpsInObject->erase(vkpsInObject->begin() + idx.at(k));
            vdescriptorsInObject->erase(vdescriptorsInObject->begin() + idx.at(k));
            vnCurrentTrackObjectID->erase(vnCurrentTrackObjectID->begin() + idx.at(k));
        }
    }

    // 3. add new track to objectBag
    for(size_t icurrentTrack = 0; icurrentTrack < vCurrentObjects.size(); icurrentTrack++)
    {
        if(vnCurrentTrackObjectID->at(icurrentTrack) == -1)
        {
            vnCurrentTrackObjectID->at(icurrentTrack) = trackId;
            trackId++;

            // update objectBag
            size_t emptyObjectBagSpace = std::count(mvObjectObservedTimes->begin(), mvObjectObservedTimes->end(), -1);
            if(emptyObjectBagSpace > 0)
            {
                for(size_t ith = 0; ith < mvObjectObservedTimes->size(); ith++)
                {
                    if(mvObjectObservedTimes->at(ith) != -1)
                        continue;
                    mvObjectObservedTimes->at(ith) = 0;
                    mvObjectUnobservedTimes->at(ith) = 0;
                    auto ORBInfo = std::make_pair(vkpsInObject->at(icurrentTrack), vdescriptorsInObject->at(icurrentTrack));
                    auto ObjectORBwithId = std::make_pair(vnCurrentTrackObjectID->at(icurrentTrack), ORBInfo);
                    mvObjectBag->at(ith) = ObjectORBwithId;
                    break;
                }
            }
            else
            {
                // erase the most times unobserved one and then replaced with the new one
                std::vector<int>::iterator biggestMissed = std::max_element(mvObjectUnobservedTimes->begin(), mvObjectUnobservedTimes->end());
                int ithMaxMissedObeject = (int)std::distance(mvObjectUnobservedTimes->begin(), biggestMissed);
                mvObjectObservedTimes->at(ithMaxMissedObeject) = 0;
                mvObjectUnobservedTimes->at(ithMaxMissedObeject) = 0;
                auto ORBInfo = std::make_pair(vkpsInObject->at(icurrentTrack), vdescriptorsInObject->at(icurrentTrack));
                auto ObjectORBwithId = std::make_pair(vnCurrentTrackObjectID->at(icurrentTrack), ORBInfo);
                mvObjectBag->at(ithMaxMissedObeject) = ObjectORBwithId;
            }
        }
    }

    // store current frame to last frame
    for(size_t i = 0; i < mvnLastTrackObjectID.size(); i++)
    {
        if(i < vCurrentObjects.size())
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

    DrawKpsWithinObject(frame, *vkpsInObject, *kpsOut);

    DrawDetector(frame, vCurrentObjects, *vnCurrentTrackObjectID);

    // store current frame information
    vframeInObjectORBpair->push_back(std::make_pair(*vkpsInObject, *vdescriptorsInObject));
    vframeOutObjectORBpair->push_back(std::make_pair(*kpsOut, *descriptorsOut));
    lastFrame = frame.clone();

    // update object Observing info, +1 or -1 for every object
    std::vector<bool>* hasBeenDealed = new std::vector<bool>(vCurrentObjects.size(), false);
    for(size_t iObjectInBag = 0; iObjectInBag < mvObjectBag->size(); iObjectInBag++)
    {
        if(mvObjectObservedTimes->at(iObjectInBag) == -1)
            continue;
        bool unObserved = false;
        int idInBag = mvObjectBag->at(iObjectInBag).first;
        for(size_t iCurrent = 0; iCurrent < vnCurrentTrackObjectID->size(); iCurrent++)
        {
            if(vnCurrentTrackObjectID->at(iCurrent) == -1 || hasBeenDealed->at(iCurrent))
                continue;

            if(vnCurrentTrackObjectID->at(iCurrent) == idInBag)
            {
                hasBeenDealed->at(iCurrent) = true;
                mvObjectObservedTimes->at(iObjectInBag) += 1;
                break;
            }
            else
            {
                if(iCurrent == vnCurrentTrackObjectID->size()-1)
                    unObserved = true;
                continue;
            }
        }
        if(unObserved)
            mvObjectUnobservedTimes->at(iObjectInBag) += 1;
    }
    // if unobserved times > 3, remove it
    for(size_t iUnOb = 0; iUnOb < mvObjectUnobservedTimes->size(); iUnOb++)
    {
        if(mvObjectUnobservedTimes->at(iUnOb) >= 5)
        {
            mvObjectObservedTimes->at(iUnOb) = -1;
            mvObjectUnobservedTimes->at(iUnOb) = -1;
        }

    }

    if (writeFrame->isOpened())
        *writeFrame << frame;

    cv::imshow("Video", frame);
    cv::waitKey(1);

    delete vnCurrentTrackObjectID;
    delete vkpsInObject;
    delete vdescriptorsInObject;
    delete kpsIn;
    delete kpsOut;
    delete descriptorsIn;
    delete descriptorsOut;

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

float ObjectTracker::Objectmatcher(Mat lastDescriptor,
                                   Mat currentDescrptor,
                                   std::vector<cv::KeyPoint> vLastKps,
                                   std::vector<cv::KeyPoint> vCurrentKps,
                                   std::vector<KeyPoint> &match1,
                                   std::vector<KeyPoint> &match2, string step, size_t &nInliers)
{
    std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();

    if(vLastKps.size() == 0 || vCurrentKps.size() == 0)
        return 0;

    std::vector<std::vector<cv::DMatch>> matches;

    matcher->knnMatch(lastDescriptor, currentDescrptor, matches, 2);


    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            match1.push_back(vLastKps[matches[i][0].queryIdx]);
            match2.push_back(vCurrentKps[matches[i][0].trainIdx]);
        }
    }

    Mat inlier_mask_F, Fundamental;
    vector<KeyPoint>  inliers1_F, inliers2_F;
    vector<DMatch> inlier_matches_F;

    if(match1.size() >= 8)
    {
        Fundamental = findFundamentalMat(Points(match1), Points(match2), FM_RANSAC, 2.5, 0.99, inlier_mask_F);
    }

//    if(match1.size() < 8) {
//        return 0;
//    }
    if(inlier_mask_F.cols * inlier_mask_F.rows == 0)
    {
         nInliers = 0;
         return 0;
    }

    for(unsigned i = 0; i < match1.size(); i++) {
        if(inlier_mask_F.at<uchar>(i)) {
            int new_i = static_cast<int>(inliers1_F.size());
            inliers1_F.push_back(match1[i]);
            inliers2_F.push_back(match1[i]);
            inlier_matches_F.push_back(DMatch(new_i, new_i, 0));
        }
    }

    std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();

    float ratio = (float)inliers1_F.size()/(float)std::min(vLastKps.size(), vCurrentKps.size());

    std::cout << step << ": match cost: "
              << std::chrono::duration_cast<std::chrono::duration<double>>(t8 - t7).count()
              << " seconds."
              << " nmatches: "
              << inliers1_F.size()
              << " ratio: "
              << ratio
              << std::endl;

    nInliers = inliers1_F.size();
//    if(inliers1_F.size() < 8)
//        return 0;

    return ratio;



}
