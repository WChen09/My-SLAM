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

#include "Map.h"

#include<mutex>
#include <sys/stat.h>
#include "Converter.h"
namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0),mnLabeledMP(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
    if(pMP->mnObjectClass > -1)
    {
        mnLabeledMP++;
    }
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    if(pMP->mnObjectClass > -1)
    {
        mnLabeledMP--;
    }
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mnLabeledMP = 0;
}


void Map::_WriteMapPoint(ofstream &f, MapPoint* mp) {
    f.write((char*)&mp->mnId, sizeof(mp->mnId));               // id: long unsigned int
    cv::Mat wp = mp->GetWorldPos();
    f.write((char*)&wp.at<float>(0), sizeof(float));           // pos x: float
    f.write((char*)&wp.at<float>(1), sizeof(float));           // pos y: float
    f.write((char*)&wp.at<float>(2), sizeof(float));           // pos z: float
}

void Map::_WriteKeyFrame(ofstream &f, KeyFrame* kf, map<MapPoint*, unsigned long int>& idx_of_mp) {
    f.write((char*)&kf->mnId, sizeof(kf->mnId));                 // id: long unsigned int
    f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));     // ts: double

#if 0
    cerr << "writting keyframe id " << kf->mnId << " ts " << kf->mTimeStamp << " frameid " << kf->mnFrameId << " TrackReferenceForFrame " << kf->mnTrackReferenceForFrame << endl;
    cerr << " parent " << kf->GetParent() << endl;
    cerr << "children: ";
    for(auto ch: kf->GetChilds())
        cerr << " " << ch->mnId;
    cerr <<endl;
    cerr << kf->mnId << " connected: (" << kf->GetConnectedKeyFrames().size() << ") ";
    for (auto ckf: kf->GetConnectedKeyFrames())
        cerr << ckf->mnId << "," << kf->GetWeight(ckf) << " ";
    cerr << endl;
#endif

    cv::Mat Tcw = kf->GetPose();
    f.write((char*)&Tcw.at<float>(0,3), sizeof(float));          // px: float
    f.write((char*)&Tcw.at<float>(1,3), sizeof(float));          // py: float
    f.write((char*)&Tcw.at<float>(2,3), sizeof(float));          // pz: float
    vector<float> Qcw = Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    f.write((char*)&Qcw[0], sizeof(float));                      // qx: float
    f.write((char*)&Qcw[1], sizeof(float));                      // qy: float
    f.write((char*)&Qcw[2], sizeof(float));                      // qz: float
    f.write((char*)&Qcw[3], sizeof(float));                      // qw: float
    f.write((char*)&kf->N, sizeof(kf->N));                       // nb_features: int
    for (int i=0; i<kf->N; i++) {
        cv::KeyPoint kp = kf->mvKeys[i];
        f.write((char*)&kp.pt.x,     sizeof(kp.pt.x));               // float
        f.write((char*)&kp.pt.y,     sizeof(kp.pt.y));               // float
        f.write((char*)&kp.size,     sizeof(kp.size));               // float
        f.write((char*)&kp.angle,    sizeof(kp.angle));              // float
        f.write((char*)&kp.response, sizeof(kp.response));           // float
        f.write((char*)&kp.octave,   sizeof(kp.octave));             // int
        for (int j=0; j<32; j++)
            f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j), sizeof(char));

        unsigned long int mpidx; MapPoint* mp = kf->GetMapPoint(i);
        if (mp == NULL) mpidx = ULONG_MAX;
        else mpidx = idx_of_mp[mp];
        f.write((char*)&mpidx,   sizeof(mpidx));                       // long int
    }

}
bool Map::Save(const string &filename)
{
    ofstream SaveMapFile(filename.c_str(), ios_base::out|ios::binary);

    cout << "  writing " << mspMapPoints.size() << " mappoints" << endl;
    unsigned long int nbMapPoints = mspMapPoints.size();
    SaveMapFile.write((char*)&nbMapPoints, sizeof(nbMapPoints));
    for(auto mp: mspMapPoints)
        _WriteMapPoint(SaveMapFile, mp);

    map<MapPoint*, unsigned long int> idx_of_mp;
    unsigned long int i = 0;
    for(auto mp: mspMapPoints) {
        idx_of_mp[mp] = i;
        i += 1;
    }

    cout << "  writing " << mspKeyFrames.size() << " keyframes" << endl;
    unsigned long int nbKeyFrames = mspKeyFrames.size();
    SaveMapFile.write((char*)&nbKeyFrames, sizeof(nbKeyFrames));
    for(auto kf: mspKeyFrames)
        _WriteKeyFrame(SaveMapFile, kf, idx_of_mp);

    // store tree and graph
    for(auto kf: mspKeyFrames) {
        KeyFrame* parent = kf->GetParent();
        unsigned long int parent_id = ULONG_MAX;
        if (parent) parent_id = parent->mnId;
        SaveMapFile.write((char*)&parent_id, sizeof(parent_id));
        unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
        SaveMapFile.write((char*)&nb_con, sizeof(nb_con));
        for (auto ckf: kf->GetConnectedKeyFrames()) {
            int weight = kf->GetWeight(ckf);
            SaveMapFile.write((char*)&ckf->mnId, sizeof(ckf->mnId));
            SaveMapFile.write((char*)&weight, sizeof(weight));
        }
    }

    SaveMapFile.close();
    cout << "Map: finished saving" << endl;
    struct stat st;
    stat(filename.c_str(), &st);
    cout << "Map: saved " << st.st_size << " bytes" << endl;

#if 0
    for(auto mp: mspMapPoints)
        if (!(mp->mnId%100))
            cerr << "mp " << mp->mnId << " " << mp->Observations() << " " << mp->isBad() << endl;
#endif

    return true;

}

} //namespace ORB_SLAM
