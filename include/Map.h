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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

//    void SetObjectMapPoints(const int n);
//    int GetALLObjectMPs();

//    void SetObjectMapPoints(const std::vector<std::vector<MapPoint *> > &vvMps);
    std::vector<std::vector<MapPoint*>> GetObjectMapPoints();
    void emptyObject();

//    void SetObjectPose(const std::vector<cv::Point3f>& vPose);
    std::vector<cv::Point3f> GetObjectPose();

    void AddObjectMapPoints(const std::vector<MapPoint*>& OMPs);
    void AddObjectPose(const cv::Point3f& p, const int Id, const std::vector<float>& range);
    void LCFreshObjectPose();
    cv::Point3f ComputeObjectPose(std::vector<MapPoint *> &MPs, cv::Point3f & PoseStd);
    void PCLStatisticalFilter(std::vector<MapPoint *> &InMps, std::vector<MapPoint *> &OutMps, const int& nNeighbors, const double& stdDevMult);
    float meadianBlurMeanStd(std::vector<float> &P, float & std);
    std::vector<std::vector<float>> GetObjectBoundingBox();
    std::vector<float> ComputeRange(std::vector<MapPoint*> & Mps);

    void SetBBoxInMap(std::vector<std::vector<cv::Point3f>> & ObjectCorners, std::vector<std::vector<MapPoint*>> & vvMps);
    void GetBBoxInMap(std::vector<std::vector<cv::Point3f>>& vvCorners, std::vector<std::vector<MapPoint*>>& vvMps);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

//    int mnLabeledMP;

    bool Save(const string &filename);
    bool Load(const string &filename, ORBVocabulary &voc, const string &settingFile);

    std::vector<std::vector<MapPoint*>> mvvObjectMapPoints;
    std::vector<cv::Point3f> mvObjectPose;
    std::vector<int> mvObjectId;
    std::vector<std::vector<float>> mvObjectBox;
    std::vector<bool> LCRemove;
    int mnObjects;
    std::vector<std::vector<cv::Point3f>> mObject2DBBox;
    std::vector<std::vector<MapPoint*>> mvvObejctMapPointTemp;


protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

    std::mutex mMutexMapTemp;

    void _WriteMapPoint(ofstream &f, MapPoint* mp);
    void _WriteKeyFrame(ofstream &f, KeyFrame* kf, map<MapPoint*, unsigned long int>& idx_of_mp);
    MapPoint*_ReadMapPoint(ifstream &f);
    KeyFrame* _ReadKeyFrame(ifstream &f, ORBVocabulary &voc, std::vector<MapPoint*> amp, ORBextractor* orb_ext);

    cv::Mat mK;
    cv::Mat mDistCoef;
};

} //namespace ORB_SLAM

#endif // MAP_H
