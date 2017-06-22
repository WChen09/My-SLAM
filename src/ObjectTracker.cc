#include "ObjectTracker.h"

// ORB Extractor parameters
const int nFeatures = 1000;
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

    frameObject = new std::vector<std::pair<int, vObjects>>();
    frameObjectID = new std::vector<std::pair<int, std::vector<int>>>();

    imgStorePath = "./result.avi";

    detector = new ORB_SLAM2::ORBextractor(nFeatures, scaleFactor, nLevels, iniFAST, minThFAST);

    matcher = new ORB_SLAM2::ORBmatcher(0.8);



    writeFrame = new cv::VideoWriter();

    if (!writeFrame->isOpened())
    {
        writeFrame->open(imgStorePath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, frameSize, true);
    }

}

ObjectTracker::~ObjectTracker()
{
    delete APS;
    delete detector;
    delete matcher;
    delete frameObject;
    delete frameObjectID;
    delete writeFrame;
}

void ObjectTracker::grabImgWithObjects(Mat &frame, vObjects &vCurrentObjects)
{
    vnCurrentTrackObjectID = new std::vector<int>(vCurrentObjects.size(), -1);

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
        auto p1 = std::make_pair(frameId, mvlastDetecedBox);
        auto p2 = std::make_pair(frameId, mvnLastTrackObjectID);
        frameObject->push_back(p1);
        frameObjectID->push_back(p2);

        DrawDetector(frame, vCurrentObjects, *vnCurrentTrackObjectID);


        if (writeFrame->isOpened())
            *writeFrame << frame;

        cv::imshow("Video", frame);
        cv::waitKey(1);
        delete vnCurrentTrackObjectID;
        frameId++;
        return;
    }


    // track last frame
    // assignment
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
    auto p1 = std::make_pair(frameId, mvlastDetecedBox);
    auto p2 = std::make_pair(frameId, mvnLastTrackObjectID);
    frameObject->push_back(p1);
    frameObjectID->push_back(p2);

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
