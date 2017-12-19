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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state
    std::vector<DetectedObject> objects;
    std::vector<int> objectsId;
    std::vector<int> MPsObjectId;
    std::vector<DetectedObject> lastObjects;
    std::vector<int> lastObjectId;
    std::vector<std::pair<cv::KeyPoint, cv::KeyPoint>> ObjectMatchKps;

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        objects = mvCurrentObjects;
        objectsId = mvObjectsId;
        MPsObjectId = mvMPsId;
        lastObjects = mvLastInCurrent;
        lastObjectId = mvLastObjectId;
        ObjectMatchKps = mvObjectMatchKps;

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
//        mnlabeled = 0;
//        mnlabeledVO = 0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    if(MPsObjectId[i]!= -1)
                    {
                        cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,255));//magenta color
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,255),-1);
                        mnTracked++;
////                        mnlabeled++;

                    }
                    else
//                    {
                        cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                        mnTracked++;
//                    }

                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    if(vCurrentKeys[i].class_id != -1)
                    {
                        cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,255));//magenta color
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,255),-1);
                        mnTrackedVO++;
//                        mnlabeledVO++;

                    }
                    else
                    {
                        cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));//red
                        cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                        mnTrackedVO++;
                    }

                }
            }
        }
        for(int i = 0; i < objects.size(); i++)
        {
            DetectedObject& o = objects[i];
            cv::rectangle(im, o.bounding_box, cv::Scalar(255,0,255), 2);

//            string class_name = names[o.object_class];

//            char str[255];
//            //sprintf(str,"%s %f", names[o.object_class], o.prob);
//            sprintf(str,"%d", objectsId.at(i));
//            cv::putText(im, str, cv::Point2f(o.bounding_box.x,o.bounding_box.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,255), 1);
        }

        for(size_t i = 0; i < lastObjects.size(); i++)
        {
            DetectedObject o = lastObjects.at(i);
            cv::rectangle(im, o.bounding_box, cv::Scalar(0,250,250), 2);

            string class_name = names[o.object_class];

            char str[255];
            //sprintf(str,"%s %f", names[o.object_class], o.prob);
            sprintf(str,"%d", lastObjectId.at(i));
            cv::putText(im, str, cv::Point2f(o.bounding_box.x,o.bounding_box.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,255), 1);
        }


        for(size_t ipair = 0; ipair < ObjectMatchKps.size(); ipair++)
        {
            cv::line(im,ObjectMatchKps.at(ipair).first.pt, ObjectMatchKps.at(ipair).second.pt, cv::Scalar(0,255,0));
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
//        int nLabeledMPs = mpMap->mnLabeledMP;
        s << "GMAP [KFs: " << nKFs << ", MPs: " << nMPs << "]  "
          << "LMAP [Matches: " << mnTracked << ", labeled Kps: " << NIn << ", ratio: "  << float(NIn)/float(N) << "]";
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    NIn = pTracker->mCurrentFrame.Nlabeled;
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
    mvCurrentObjects = pTracker->mCurrentFrame.mvObjects;
    mvObjectsId = pTracker->mCurrentFrame.mvObjectId;
    mvMPsId = pTracker->mCurrentFrame.mvMapPointsId;
    mvLastInCurrent = pTracker->mLastFrame.mvLastObjectProInCurrent;
    mvLastObjectId = pTracker->mLastFrame.mvObjectId;

    mvObjectMatchKps = pTracker->mvObjectMatchKps;

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

void FrameDrawer::loadObjectNames(vector<string> names_){
    names = names_;
}

} //namespace ORB_SLAM
