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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>

#include "unistd.h"
#include<assert.h>

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    if(pMap->KeyFramesInMap() == 0)
        mState = NO_IMAGES_YET;
    else{
        mState = LOST;
        mbOnlyTracking = true;
        std::vector<KeyFrame*> akf = pMap->GetAllKeyFrames();
        mpReferenceKF = *akf.end();
        mlRelativeFramePoses.push_back(cv::Mat::eye(4,4,CV_32FC1));
    }

    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }


    nMPsMatchedTh = fSettings["nMPsMatchedTh"];
    std::cout << nMPsMatchedTh << std::endl;
    nObjects = 0;

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, const std::vector<DetectedObject> &objects)
{
    mImGray = im;

    //if object area over threshold obth, return last pose
    //    float obth = 0.3;
    int wholeArea = 0, objectArea = 0;
    wholeArea = im.cols * im.rows;
    std::cout << " detect " << objects.size() << " objects: " << "[class, prob, areaRatio]" << " ";
    for(int i = 0; i < objects.size(); i++){
        DetectedObject currO = objects[i];
        objectArea += currO.bounding_box.area();
        std::cout << "  [" << currO.object_class << ", " << currO.prob << ", " << float(currO.bounding_box.area())/float(wholeArea) << "] ";
    }
    //    std::cout << std::endl << "whole Area ratio: " << float(objectArea)/float(wholeArea) << endl;
    //        if(objectArea/wholeArea > obth){
    //            return mLastFrame.mTcw.clone();
    //        }

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth, objects);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth, objects);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    cout << "new image " << mCurrentFrame.mnId << endl;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK){
                        bOK = TrackReferenceKeyFrame();
                    }
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated
            std::cout << "Localization Mode" << std::endl;

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
            if(!bOK)
                cout << "Track lost because of TrackLocalMap" << endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        if(mSensor != System::MONOCULAR)
            mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
            {
                cout << "mLastFrame.mTcw is empty" << endl;
                mVelocity = cv::Mat();
            }
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // for monocular case, mlpTemporalPoints is empty
            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Object tracking
        if(mState == OK)
        {
            ObjectTracking(); // update MPs object id
            updateObjectMapPoints();
//            mpMap->SetObjectMapPoints(nObjects);
            mpMap->SetObjectMapPoints(mvvObjectMps);
            mpMap->SetObjectPose(mvObjectPose);
        }
        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mpFrameDrawer->Update(this);

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse(); //Tcr  = Tcw(c) * Twc(r)
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }



}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        //        // set label to MapPoints
        //        const cv::KeyPoint &kp2 = pKFcur->mvKeysUn[i];
        //        if(kp2.class_id != -1){
        //            pMP->setLabelClass(kp2.class_id);
        //        }

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    cerr << "into TrackReferenceKeyFrame" << endl;
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    cout << "nmatches: " << nmatches << endl;
    if(nmatches<15)
        return false;
    cout << "TrackReferenceKeyFrame nmatches: " << nmatches << endl;
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=15;//change from 7 to 15
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);
    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);// get MapPoints for current frame
    }
    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches, and find out outlire MapPoints
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }
    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints(); // the second place to add MapPoints for current frame

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<20)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;//mnMatchesInliers is matched mappoints of current frame in local map
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }
    cout << "SearchLocalPoints find nToMatch: " << nToMatch << endl;
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();

    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);


}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            /*            //culling labeled MPs
            if(pMP->mnObjectClass != -1)
                continue*/;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();
    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);
    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }

            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }

        }
    }
    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "localization done!" << endl;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);//3seconds
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::ObjectTracking()
{
    if(mCurrentFrame.mvObjects.empty() && mLastFrame.mvObjects.empty())
        return;

    if(nObjects == 0)
    {
        // get matching MapPoints for current frame, and change these MapPoints' class_id and object id
        FirstTrack();
        return;
    }
    else
    {

        cv::Mat Tcl, Tlc;
        computeTInLast(Tcl,Tlc);

        ProjectLastObjectsInCurrent(Tcl, mLastFrame.mvvObjectBoxCornerLocationInFrame, mLastFrame.mvObjects, mLastFrame.mvLastObjectProInCurrent);

        std::cout << mLastFrame.mvLastObjectProInCurrent.size() << std::endl;

        size_t N = mLastFrame.mvLastObjectProInCurrent.size();
        size_t M = mCurrentFrame.mvObjects.size();

        std::vector<int>* assignment = new std::vector<int>(N, -1);

        std::cout << "assignment: " ;
        for(size_t i = 0; i < N; i++)
        {
            std::cout << assignment->at(i) << " ";
        }
        std::cout <<std::endl;

        HungarianAssignment(*assignment, mLastFrame.mvLastObjectProInCurrent, mCurrentFrame.mvObjects, 0.8);

        std::cout << "assignment: " ;
        for(size_t i = 0; i < N; i++)
        {
            std::cout << assignment->at(i) << " ";
        }
        std::cout <<std::endl;

        //        if(N>=M)
        TrackLastObjects(*assignment, nMPsMatchedTh, mLastFrame.mvLastObjectProInCurrent);

        UpdateTrackObject(*assignment);

        std::cout << "Current frame objects: " << mCurrentFrame.mvObjects.size() << std::endl;
        std::cout << "Objects in bag: " << mvvObjectMps.size() << std::endl;

        delete assignment;

    }

}

void Tracking::TrackLastObjects(std::vector<int> &_assignment, const float nMatchesRatio, std::vector<DetectedObject> &lastObjectProInCurrent)
{

    for(int iLastObject = 0;
        iLastObject < lastObjectProInCurrent.size(); iLastObject++)
    {
        if(_assignment.at(iLastObject)!=-1)
            continue;
        DetectedObject lastObject = lastObjectProInCurrent.at(iLastObject);
        // find MPs in current frame using lastObject
        std::vector<MapPoint*> vMPsLastInCurrent;
        for(size_t iMP = 0; iMP < mCurrentFrame.mvpMapPoints.size(); iMP++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints.at(iMP);
            if(pMP)
            {
                if(lastObject.bounding_box.contains(mCurrentFrame.mvKeysUn.at(iMP).pt))
                    vMPsLastInCurrent.push_back(pMP);
            }
        }

        int nCurrentMps = vMPsLastInCurrent.size();
        int nLastMps = mLastFrame.mvObjectMPs.at(iLastObject).size();
        int nMatches = 0;
        std::vector<bool> alreadyMatched(nCurrentMps, false);
        for(size_t iL = 0; iL < nLastMps; iL++)
        {
            for(size_t iC = 0; iC < nCurrentMps; iC++)
            {
                if(alreadyMatched.at(iC))
                    continue;

                if(mLastFrame.mvObjectMPs.at(iLastObject).at(iL) == vMPsLastInCurrent.at(iC))
                {
                    alreadyMatched.at(iC) = true;
                    nMatches++;
                    break;
                }
                else
                    continue;

            }
        }

        float nMatchRatio = (float)nMatches/(float)(0.5*nCurrentMps+0.5*nLastMps);

        // < means Lost or have leave current view
        if(nMatchRatio < nMatchesRatio)
            continue;
        else
        {
            //update this object and its ID to current frame
            mCurrentFrame.mvObjects.push_back(lastObject);
            _assignment.at(iLastObject) = mCurrentFrame.mvObjects.size()-1;
        }
    }
}

void Tracking::UpdateTrackObject(std::vector<int>& _assignment)
{

    mCurrentFrame.mvObjectId.resize(mCurrentFrame.mvObjects.size());
    for(size_t i = 0; i < _assignment.size(); i++)
    {
        if(_assignment.at(i)!=-1)
            mCurrentFrame.mvObjectId.at(_assignment.at(i)) = mLastFrame.mvObjectId.at(i);
    }

    for(size_t i = 0; i < mCurrentFrame.mvObjectId.size(); i++)
    {
        if(mCurrentFrame.mvObjectId.at(i) == -1)
        {
            mCurrentFrame.mvObjectId.at(i) = nObjects;
            nObjects++;
        }
    }

    //add object id to Mps
    std::vector<std::vector<MapPoint*>> ObjectMPs(mCurrentFrame.mvObjects.size());
    for(size_t iMp = 0; iMp < mCurrentFrame.mvpMapPoints.size(); iMp++)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints.at(iMp);
        if(pMP)
        {
            for(size_t iObject = 0; iObject < mCurrentFrame.mvObjects.size(); iObject++)
            {
                if(mCurrentFrame.mvObjects.at(iObject).bounding_box.contains(mCurrentFrame.mvKeysUn.at(iMp).pt))
                {

                    mCurrentFrame.mvpMapPoints.at(iMp)->setObjectId(mCurrentFrame.mvObjectId.at(iObject));
                    mCurrentFrame.mvpMapPoints.at(iMp)->setClassId(mCurrentFrame.mvObjects.at(iObject).object_class);
                    mCurrentFrame.mvMapPointsId.at(iMp) = mCurrentFrame.mvObjectId.at(iObject);

                    ObjectMPs.at(iObject).push_back(pMP);
                    //                        std::cout << mCurrentFrame.mvMapPointsId.at(iMp) << " ";
                }
            }
        }
    }

    // remove the background MPs
    std::vector<cv::Point3f> ObjectsPose(ObjectMPs.size());
    for(size_t i = 0; i < ObjectMPs.size(); i++)
    {
        std::vector<MapPoint*> iObjectMPs = ObjectMPs.at(i);
        ObjectsPose.at(i) = ComputeObjectPose(iObjectMPs);
    }

    // depth
    std::vector<float> ObjectDepth(ObjectMPs.size(), 0);
    for(size_t iObject = 0; iObject < ObjectMPs.size(); iObject++)
    {
        float Depth = 0;
        ComputeObjectDepth(ObjectMPs.at(iObject), Depth); //need var limination
        ObjectDepth.at(iObject) = Depth;
        std::cout << Depth << " ";
    }

    std::vector<std::vector<cv::Mat>> ObjectBoxCornerLocation;
    ComputeObjectBoxCorner(mCurrentFrame.mvObjects, ObjectBoxCornerLocation, ObjectDepth);

    mCurrentFrame.mvvObjectBoxCornerLocationInFrame = ObjectBoxCornerLocation;
    mCurrentFrame.mvObjectDepth = ObjectDepth;
    mCurrentFrame.mvObjectMPs = ObjectMPs;
    mCurrentFrame.mvObjectPose = ObjectsPose;

    //std::cout << std::endl;
}

void Tracking::FirstTrack()
{
    /*
        1. add these Mps to current frame's object
    */

    //add object Id
    for(size_t iObject = 0; iObject < mCurrentFrame.mvObjects.size(); iObject++)
    {
        mCurrentFrame.mvObjectId.at(iObject) = nObjects;
        nObjects++;
        //        std::cout << mCurrentFrame.mvObjectId.at(iObject);
    }
    //    std::cout << std::endl;

    //add object id to Mps
    std::vector<std::vector<MapPoint*>> ObjectMPs(mCurrentFrame.mvObjects.size());

    for(size_t iMp = 0; iMp < mCurrentFrame.mvpMapPoints.size(); iMp++)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints.at(iMp);
        if(pMP)
        {
            for(size_t iObject = 0; iObject < mCurrentFrame.mvObjects.size(); iObject++)
            {
                if(mCurrentFrame.mvObjects.at(iObject).bounding_box.contains(mCurrentFrame.mvKeysUn.at(iMp).pt))
                {
//                    mCurrentFrame.mvpMapPoints.at(iMp)->setObjectId(mCurrentFrame.mvObjectId.at(iObject));
//                    mCurrentFrame.mvpMapPoints.at(iMp)->setClassId(mCurrentFrame.mvObjects.at(iObject).object_class);
//                    mCurrentFrame.mvMapPointsId.at(iMp) = mCurrentFrame.mvObjectId.at(iObject);
                    //                    std::cout << mCurrentFrame.mvMapPointsId.at(iMp) << " ";
                    ObjectMPs.at(iObject).push_back(pMP);
                }
            }
        }
    }
    //    std::cout << std::endl;

    // --- remove the background MPs
    // 1. compute the center of objects
    std::vector<cv::Point3f> ObjectsPose(ObjectMPs.size());
    for(size_t i = 0; i < ObjectMPs.size(); i++)
    {
        std::vector<MapPoint*> iObjectMPs = ObjectMPs.at(i);
        ObjectsPose.at(i) = ComputeObjectPose(iObjectMPs);

        std::vector<MapPoint*> iObjectMPs_filter;
        PCLStatisticalFilter(iObjectMPs, iObjectMPs_filter);
    }
    // 2. PCL remove



    // depth
    std::vector<float> ObjectDepth(ObjectMPs.size(), 0);
    for(size_t iObject = 0; iObject < ObjectMPs.size(); iObject++)
    {
        float Depth = 0;
        ComputeObjectDepth(ObjectMPs.at(iObject), Depth); //add medianBlur
        ObjectDepth.at(iObject) = Depth;
        std::cout << Depth << " ";

    }

    std::vector<std::vector<cv::Mat>> ObjectBoxCornerLocation;
    ComputeObjectBoxCorner(mCurrentFrame.mvObjects, ObjectBoxCornerLocation, ObjectDepth);

    //    std::cout << "Box corners: " << std::endl;
    //    for(int i = 0; i < ObjectBoxCornerLocation.size(); i++)
    //    {
    //        std::cout << std::endl;
    //        for(int j = 0; j < ObjectBoxCornerLocation.at(i).size(); j++)
    //        {
    //            std::cout << ObjectBoxCornerLocation.at(i).at(j) << " " << std::endl;
    //        }
    //    }
    //    std::cout << std::endl;

    mCurrentFrame.mvvObjectBoxCornerLocationInFrame = ObjectBoxCornerLocation;
    mCurrentFrame.mvObjectDepth = ObjectDepth;
    mCurrentFrame.mvObjectMPs = ObjectMPs;
    mCurrentFrame.mvObjectPose = ObjectsPose;

}

void Tracking::PCLStatisticalFilter(std::vector<MapPoint *> &InMps, std::vector<MapPoint *> &OutMps)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = (int)InMps.size();
    cloud->height = 1;
    
    for(size_t iMp = 0; iMp < InMps.size(); iMp++)
    {
        MapPoint* mp = InMps.at(iMp);
        cv::Mat x3Dw = mp->GetWorldPos();
        cloud->points.push_back(pcl::PointXYZ(x3Dw.at<float>(0),x3Dw.at<float>(1),x3Dw.at<float>(2)));
//        cloud->points.at(iMp).x = x3Dw.at<float>(0);
//        cloud->points.at(iMp).y = x3Dw.at<float>(1);
//        cloud->points.at(iMp).z = x3Dw.at<float>(2);
    }

//    std::cout << *cloud << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK(5);
    sor.setStddevMulThresh(1.0);
    pcl::PointCloud<pcl::PointXYZ> cloudOut;
    sor.setNegative (true);
    sor.filter(cloudOut);
//    std::cout << cloudOut << std::endl;
    std::vector<pcl::PointXYZ> outliers = cloudOut.points;
    std::vector<bool> judged(outliers.size(), false);
    std::vector<pcl::PointXYZ> in = cloud->points;
    for(size_t i = 0; i < in.size(); i++)
    {
        pcl::PointXYZ inP = in.at(i);
        for(size_t j = 0; j < outliers; j++)
        {
//              if(inP. ==)
        }
    }

}

cv::Point3f Tracking::ComputeObjectPose(std::vector<MapPoint *> &MPs)
{
    if(MPs.size() == 0)
    {
        return cv::Point3f(0,0,0);
    }

    std::vector<float> x(MPs.size(), 0);
    std::vector<float> y(MPs.size(), 0);
    std::vector<float> z(MPs.size(), 0);

    for(int iMp = 0; iMp < MPs.size(); iMp++)
    {
        MapPoint* pMp = MPs.at(iMp);
        cv::Mat x3Dw = pMp->GetWorldPos();
        x.at(iMp) = x3Dw.at<float>(0);
        y.at(iMp) = x3Dw.at<float>(1);
        z.at(iMp) = x3Dw.at<float>(2);
    }

    float xMean = meadianBlurMean(x);
    float yMean = meadianBlurMean(y);
    float zMean = meadianBlurMean(z);

    return cv::Point3f(xMean, yMean, zMean);
}

float Tracking::meadianBlurMean(std::vector<float> &P)
{

    cv::Mat mP(P.size(), 1, CV_32F);
    for(size_t i = 0; i < P.size(); i++)
    {
        mP.at<float>(i) = P.at(i);
    }
    cv::Mat mPBlur;
    cv::medianBlur(mP, mPBlur, 3);

    cv::Mat m;
    cv::meanStdDev(mPBlur, m, cv::Mat());

    return m.at<double>(0);
}

void Tracking::computeTInLast(cv::Mat &Tcl, cv::Mat &Tlc)
{
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3); //3x3
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0,3).col(3); //3x1

    const cv::Mat twc = -Rcw.t()*tcw; // 3x1  R_INV = R.t, but T_INV != T.t

    const cv::Mat Rlw = mLastFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = mLastFrame.mTcw.rowRange(0,3).col(3);
    //vector from LastFrame to CurrentFrame expressed in LastFrame
    const cv::Mat tlc = Rlw*twc+tlw;// Rlw*twc(w) = twc(l), tlc(l) = twc(l) + tlw(l)
    const cv::Mat Rlc = Rlw*Rcw.t();

    Tlc = cv::Mat::eye(4,4,mCurrentFrame.mTcw.type());
    Tcl = cv::Mat::eye(4,4,mCurrentFrame.mTcw.type());

    Rlc.copyTo(Tlc.rowRange(0,3).colRange(0,3));
    tlc.copyTo(Tlc.rowRange(0,3).col(3));

    const cv::Mat tcl = -Rlc.t()*tlc;
    const cv::Mat Rcl = Rlc.t();

    tcl.copyTo(Tcl.rowRange(0,3).col(3));
    Rcl.copyTo(Tcl.rowRange(0,3).colRange(0,3));
}

void Tracking::ComputeObjectDepth(const std::vector<MapPoint *> MPs, float & depth)
{
    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3); //3x3
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0,3).col(3); //3x1

    std::vector<float> vDepth(MPs.size(), -1);

    //    std::cout << "Depth " << " ";
    for(int iMp = 0; iMp < MPs.size(); iMp++)
    {
        MapPoint* pMp = MPs.at(iMp);
        cv::Mat x3Dw = pMp->GetWorldPos();
        cv::Mat x3Dc = Rcw*x3Dw+tcw;

        vDepth.at(iMp) = x3Dc.at<float>(2);
        //        std::cout << x3Dc.at<float>(2) << " ";
    }
    //    std::cout << std::endl;
    if(MPs.size() == 0)
    {
        depth = -1;
        return;
    }
    else if(MPs.size() == 1)
    {
        depth = vDepth.at(0);
        return;
    }
    else if(MPs.size() < 3)
    {
        depth = (vDepth.at(0) + vDepth.at(1))/2.0;
        return;
    }

    //convert vector -> Mat
    cv::Mat mDepth(vDepth.size(), 1, Rcw.type());
    for(size_t i = 0; i < vDepth.size(); i++)
    {
        mDepth.at<float>(i) = vDepth.at(i);
    }
    cv::Mat mDepthBlur;
    cv::medianBlur(mDepth, mDepthBlur, 3);
//    std::cout << mDepth << std::endl
//              << mDepthBlur << std::endl;
//    depth = cv::mean(mDepthBlur);
    cv::Mat m;
    cv::meanStdDev(mDepthBlur, m, cv::Mat());

//    std::cout << m << std::endl;
    depth = m.at<double>(0);
//    std::vector<float> vDepthSorted;

//    cv::sort(vDepth, vDepthSorted, CV_SORT_ASCENDING);


//    if(vDepthSorted.size()%2 == 0)
//        depth = vDepthSorted.at(vDepthSorted.size()/2);
//    else
//    {
//        depth = (vDepthSorted.at(cvFloor((float)vDepthSorted.size()/2))
//                 + vDepthSorted.at(cvCeil((float)vDepthSorted.size()/2)))/2;
//    }
}

void Tracking::ComputeObjectBoxCorner(std::vector<DetectedObject> &Objects, std::vector<std::vector<cv::Mat> > &CornerPose, std::vector<float> &Depth)
{
    float invfx = mCurrentFrame.invfx;
    float invfy = mCurrentFrame.invfy;
    float cx = mCurrentFrame.cx;
    float cy = mCurrentFrame.cy;

    CornerPose.resize(Objects.size());

    for(size_t i = 0; i < Objects.size(); i++)
    {
        DetectedObject object =  mCurrentFrame.mvObjects.at(i);
        std::vector<cv::Point> corners;
        cv::Rect box = object.bounding_box;
        cv::Point tl = box.tl();
        cv::Point tr(tl.x + box.width, tl.y);
        cv::Point br = box.br();
        cv::Point bl(br.x-box.width, br.y);
        corners.push_back(tl);
        corners.push_back(tr);
        corners.push_back(br);
        corners.push_back(bl);

        CornerPose.at(i).resize(4);

        float z = Depth.at(i);

        for(int iC = 0; iC < corners.size(); iC++)
        {
            const float u = corners[iC].x;
            const float v = corners[iC].y;
            const float x = (u-cx)*z*invfx;
            const float y = (v-cy)*z*invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
            CornerPose.at(i).at(iC) = x3Dc;
        }
    }
}

void Tracking::ProjectLastObjectsInCurrent(const cv::Mat Tcl, std::vector<std::vector<cv::Mat>> &LastObjectsBox, std::vector<DetectedObject> &_LastObjects, std::vector<DetectedObject> &_LastObjectsInCurrent)
{
    float fx = mCurrentFrame.fx;
    float fy = mCurrentFrame.fy;
    float cx = mCurrentFrame.cx;
    float cy = mCurrentFrame.cy;

    //    _LastObjectsInCurrent.resize(LastObjectsBox.size());

    //    std::cout << "Project: " << std::endl;
    //    std::cout << fx << " " << fy << " " << cx << " " << cy << std::endl;
    //    std::cout << Tcl << std::endl;

    std::vector<DetectedObject> lastIncurrent(LastObjectsBox.size());
    for(size_t iBox = 0; iBox < LastObjectsBox.size(); iBox++)
    {
        std::vector<cv::Mat> FourCorners = LastObjectsBox.at(iBox);

        // corners in current frame
        std::vector<cv::Point2f> corners(4);
        for(size_t iCorner = 0; iCorner < FourCorners.size(); iCorner++)
        {
            const cv::Mat x3Dl = FourCorners.at(iCorner);
            cv::Mat x3Dl4  = cv::Mat::ones(4,1, x3Dl.type());
            x3Dl.copyTo(x3Dl4.rowRange(0,3));
            cv::Mat x3Dc4 = (Tcl*x3Dl4);
            const cv::Mat x3Dc = x3Dc4.rowRange(0,3);
            const float x = x3Dc.at<float>(0);
            const float y = x3Dc.at<float>(1);
            const float z = x3Dc.at<float>(2);
            const float u = fx*x/z + cx;
            const float v = fy*y/z + cy;
            cv::Point2f p(u,v);
            corners.at(iCorner) = p;
        }

        // find a Rect construct these corners to a DetectedObject
        cv::Point2f tlf = corners.at(0);
        cv::Point2f trf = corners.at(1);
        cv::Point2f brf = corners.at(2);
        cv::Point2f blf = corners.at(3);

        //        std::cout << "Project four corners: " << tlf << trf << brf << blf << std::endl;

        int xl = cvRound((tlf.x + blf.x)/2.0);
        int xr = cvRound((trf.x + brf.x)/2.0);
        int yt = cvRound((tlf.y + trf.y)/2.0);
        int yb = cvRound((blf.y + brf.y)/2.0);

        int width = xr - xl;
        int height = yb - yt;

        cv::Rect boxPr(xl,yt, width,height);
        //        cv::Rect boxlast = _LastObjects.at(iBox).bounding_box;
        //        std::cout << "project last box" << boxlast << " -> " << boxPr << std::endl;
        DetectedObject objectPr(_LastObjects.at(iBox).object_class, _LastObjects.at(iBox).prob, boxPr);

        lastIncurrent.at(iBox) = objectPr;
    }

    // remove some boxes which have leave current frame
    for(size_t iBox = 0; iBox < lastIncurrent.size(); iBox++)
    {
        DetectedObject lastBox = lastIncurrent.at(iBox);
        cv::Point br = lastBox.bounding_box.br();
        cv::Point tl = lastBox.bounding_box.tl();
        cv::Rect viewBox = cv::Rect(cv::Point(0,0), cv::Size(mImGray.cols, mImGray.rows));
        float areaRate = (float) (lastBox.bounding_box & viewBox).area()/(float)(lastBox.bounding_box.area());
        if((tl.x < 0 || br.x > 0) && areaRate < 0.5)
            continue;
        else
            _LastObjectsInCurrent.push_back(lastIncurrent.at(iBox));
    }
}

void Tracking::updateObjectMapPoints()
{
    if(nObjects == 0)
        return;
    else
    {
        for(size_t iC = 0; iC < mCurrentFrame.mvObjectMPs.size(); iC++)
        {
            if(msObjectId.count(mCurrentFrame.mvObjectId.at(iC)))
            {
                size_t iObjectInBag = std::distance(msObjectId.begin(),
                                                    msObjectId.find(mCurrentFrame.mvObjectId.at(iC)));
                std::set<MapPoint*> sExitingMps(mvvObjectMps.at(iObjectInBag).begin(), mvvObjectMps.at(iObjectInBag).end());

                for(size_t iMPs = 0; iMPs < mCurrentFrame.mvObjectMPs.at(iC).size(); iMPs++)
                {
                    if(!sExitingMps.count(mCurrentFrame.mvObjectMPs.at(iC).at(iMPs)))
                    {
                        mvvObjectMps.at(iObjectInBag).push_back(mCurrentFrame.mvObjectMPs.at(iC).at(iMPs));
                    }
                    else
                        continue;
                }
                mvObjectObserveTimes.at(iObjectInBag) += 1;

            }
            else
            {
                msObjectId.insert(mCurrentFrame.mvObjectId.at(iC));
                mvvObjectMps.push_back(mCurrentFrame.mvObjectMPs.at(iC));
                mvObjectObserveTimes.push_back(1);
                mvObjectPose.push_back(mCurrentFrame.mvObjectPose.at(iC));
            }
        }
    }
}
} //namespace ORB_SLAM
