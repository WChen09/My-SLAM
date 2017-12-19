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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    // draw global map points (black)
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]) || vpMPs[i]->mnObjectId != -1)
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    // draw local map points (red)
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad() || (*sit)->mnObjectId != -1)
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad() || (*sit)->mnObjectId == -1)
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
    glEnd();

}

void MapDrawer::DrawObjectMapPoints()
{
    //draw object centre point(red)
    const std::vector<cv::Point3f> vObjectPose = mpMap->GetObjectPose();
    //    std::cout << "Object In Map " << vObjectPose.size() << std::endl;
    glPointSize(mPointSize*5);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for(size_t i = 0; i < vObjectPose.size(); i++)
    {
        cv::Point3f p = vObjectPose.at(i);
        glVertex3f(p.x, p.y, p.z);
        //        std::cout << p << std::endl;
    }

    glEnd();

//    //draw object map points(green)
//    glPointSize(mPointSize*2);
//    glBegin(GL_POINTS);
//    glColor3f(0.0,1.0,0.0);
//    const std::vector<std::vector<MapPoint*>> vvObjectMps = mpMap->GetObjectMapPoints();

//    for(size_t i = 0; i < vvObjectMps.size(); i++)
//    {
//        std::vector<MapPoint*> iObjectMps = vvObjectMps.at(i);
//        for(size_t j = 0; j < iObjectMps.size(); j++)
//        {
//            MapPoint* pMp = iObjectMps.at(j);
//            if(pMp->isBad())
//                continue;
//            cv::Mat pos = pMp->GetWorldPos();
//            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//        }
//    }
//    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::DrawObjectBox()
{

    std::vector<std::vector<float>> ObjectRange = mpMap->GetObjectBoundingBox();
    std::vector<bool> LCRemove = mpMap->LCRemove;

    for(size_t i = 0; i < ObjectRange.size(); i++)
    {
        if(LCRemove.at(i))
            continue;

        std::vector<float> axRange = ObjectRange.at(i);
        glLineWidth(mGraphLineWidth);
        glColor3f(0.0f,0.0f,1.0f);
        glBegin(GL_LINES);
        const float minx = axRange.at(0);
        const float maxx = axRange.at(1);
        const float miny = axRange.at(2);
        const float maxy = axRange.at(3);
        const float minz = axRange.at(4);
        const float maxz = axRange.at(5);

        // bottom plane
        glVertex3f(minx, miny, minz);
        glVertex3f(maxx, miny, minz);

        glVertex3f(maxx, miny, minz);
        glVertex3f(maxx, maxy, minz);

        glVertex3f(maxx, maxy, minz);
        glVertex3f(minx, maxy, minz);

        glVertex3f(minx, maxy, minz);
        glVertex3f(minx, miny, minz);

        // upper plane
        glVertex3f(minx, miny, maxz);
        glVertex3f(maxx, miny, maxz);

        glVertex3f(maxx, miny, maxz);
        glVertex3f(maxx, maxy, maxz);

        glVertex3f(maxx, maxy, maxz);
        glVertex3f(minx, maxy, maxz);

        glVertex3f(minx, maxy, maxz);
        glVertex3f(minx, miny, maxz);

        // four lines
        glVertex3f(minx, miny, minz);
        glVertex3f(minx, miny, maxz);

        glVertex3f(maxx, miny, minz);
        glVertex3f(maxx, miny, maxz);

        glVertex3f(maxx, maxy, minz);
        glVertex3f(maxx, maxy, maxz);

        glVertex3f(minx, maxy, minz);
        glVertex3f(minx, maxy, maxz);

        glEnd();
    }

    //draw 2d object box in map
    std::vector<std::vector<MapPoint*>> vvMps;
    std::vector<std::vector<cv::Point3f>> vvCorners;
    mpMap->GetBBoxInMap(vvCorners, vvMps);

    if(vvMps.size() != 0)
    {
    	//MapPoints
    	glPointSize(mPointSize*2);
    	glBegin(GL_POINTS);
    	glColor3f(0.0,1.0,0.0);

    	for(size_t i = 0; i < vvMps.size(); i++)
    	{
    		for(size_t j = 0; j < vvMps.at(i).size(); j++)
    		{
    			MapPoint* iMp = vvMps.at(i).at(j);
    			if(iMp)
    			{
    		        cv::Mat pos = iMp->GetWorldPos();
    		        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    			}
    		}
    	}
    	glEnd();

    	// 2D bounding box
    	for(size_t iBox = 0; iBox < vvCorners.size(); iBox++)
    	{
            glLineWidth(mGraphLineWidth);
            glColor3f(0.0f,1.0f,0.0f);
            glBegin(GL_LINES);

            glVertex3f(vvCorners.at(iBox).at(0).x, vvCorners.at(iBox).at(0).y, vvCorners.at(iBox).at(0).z);
            glVertex3f(vvCorners.at(iBox).at(1).x, vvCorners.at(iBox).at(1).y, vvCorners.at(iBox).at(1).z);

            glVertex3f(vvCorners.at(iBox).at(1).x, vvCorners.at(iBox).at(1).y, vvCorners.at(iBox).at(1).z);
            glVertex3f(vvCorners.at(iBox).at(2).x, vvCorners.at(iBox).at(2).y, vvCorners.at(iBox).at(2).z);

            glVertex3f(vvCorners.at(iBox).at(2).x, vvCorners.at(iBox).at(2).y, vvCorners.at(iBox).at(2).z);
            glVertex3f(vvCorners.at(iBox).at(3).x, vvCorners.at(iBox).at(3).y, vvCorners.at(iBox).at(3).z);

            glVertex3f(vvCorners.at(iBox).at(3).x, vvCorners.at(iBox).at(3).y, vvCorners.at(iBox).at(3).z);
            glVertex3f(vvCorners.at(iBox).at(0).x, vvCorners.at(iBox).at(0).y, vvCorners.at(iBox).at(0).z);

            glEnd();
    	}

    }



}

} //namespace ORB_SLAM
