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

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0), mnObjects(0) //,mnLabeledMP(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexMap);
	mspKeyFrames.insert(pKF);
	if(pKF->mnId>mnMaxKFid)
		mnMaxKFid=pKF->mnId;
}

//void Map::SetObjectMapPoints(const int n)
//{
//    unique_lock<mutex> lock(mMutexMap);
//    mnObjects = n;
//}


//int Map::GetALLObjectMPs()
//{   unique_lock<mutex> lock(mMutexMap);
//    return  mnObjects;
//}
//void Map::SetObjectPose(const std::vector<cv::Point3f> &vPose)
//{
//    unique_lock<mutex> lock(mMutexMap);
//    mvObjectPose = vPose;
//}

void Map::PCLStatisticalFilter(std::vector<MapPoint *> &InMps, std::vector<MapPoint *> &OutMps, const int& nNeighbors, const double& stdDevMult)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    cloud->width = (int)InMps.size();
    cloud->height = 1;
    for(size_t iMp = 0; iMp < InMps.size(); iMp++)
    {
        MapPoint* mp = InMps.at(iMp);
        cv::Mat x3Dw = mp->GetWorldPos();
        cloud->points.push_back(pcl::PointXYZ(x3Dw.at<float>(0),x3Dw.at<float>(1),x3Dw.at<float>(2)));
    }
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(nNeighbors);
    sor.setStddevMulThresh(stdDevMult);
    std::vector<int> removeId;
    sor.setNegative (true);
    sor.filter(removeId);

    std::set<int> OutliersId;
    for(size_t i = 0; i < removeId.size(); i++)
    {
        OutliersId.insert(removeId.at(i));
    }

    OutMps.reserve(InMps.size());
    for(size_t iMp = 0; iMp < InMps.size(); iMp++)
    {
        if(OutliersId.count(iMp))
            continue;
        MapPoint* mp = InMps.at(iMp);
        OutMps.push_back(mp);
    }


}

float Map::meadianBlurMeanStd(std::vector<float> &P, float & std)
{

    cv::Mat mP(P.size(), 1, CV_32F);
    for(size_t i = 0; i < P.size(); i++)
    {
        mP.at<float>(i) = P.at(i);
    }
    cv::Mat mPBlur;
    cv::medianBlur(mP, mPBlur, 3);

    cv::Mat m;
    cv::Mat stdm;
    cv::meanStdDev(mPBlur, m, stdm);
    std = stdm.at<double>(0);
    return m.at<double>(0);
}

cv::Point3f Map::ComputeObjectPose(std::vector<MapPoint *> &MPs, cv::Point3f & PoseStd)
{
    if(MPs.size() == 0)
    {
        return cv::Point3f(0,0,0);
    }

    std::vector<float> x(MPs.size(), 0);
    std::vector<float> y(MPs.size(), 0);
    std::vector<float> z(MPs.size(), 0);

    for(size_t iMp = 0; iMp < MPs.size(); iMp++)
    {
        MapPoint* pMp = MPs.at(iMp);
        cv::Mat x3Dw = pMp->GetWorldPos();
        x.at(iMp) = x3Dw.at<float>(0);
        y.at(iMp) = x3Dw.at<float>(1);
        z.at(iMp) = x3Dw.at<float>(2);
    }
    float xStd, yStd, zStd;
    float xMean = meadianBlurMeanStd(x, xStd);
    float yMean = meadianBlurMeanStd(y, yStd);
    float zMean = meadianBlurMeanStd(z, zStd);

    PoseStd = cv::Point3f(xStd, yStd, zStd);

    return cv::Point3f(xMean, yMean, zMean);
}


void Map::LCFreshObjectPose()
{
	unique_lock<mutex> lock(mMutexMap);
	for(size_t iObject = 0; iObject < mvvObjectMapPoints.size(); iObject++)
	{

		std::vector<MapPoint*> ObjectMps = mvvObjectMapPoints.at(iObject);

        std::vector<MapPoint*> vObjectMPsFilter1;
        PCLStatisticalFilter(ObjectMps , vObjectMPsFilter1, (int)(ObjectMps .size()/1), 0.01);
//            std::cout << "add object " << DoneObjectId << " " << vDoneObjectMPs.size() << " -> " << vObjectMPsFilter.size() <<  std::endl;
        cv::Point3f PoseStd1;
        cv::Point3f ObjectPose1;
        ObjectPose1 = ComputeObjectPose(vObjectMPsFilter1, PoseStd1);
        std::cout << "Box Std: " << cv::norm(PoseStd1) << std::endl;

        std::vector<MapPoint*> vObjectMPsFilter2;
        PCLStatisticalFilter(ObjectMps , vObjectMPsFilter2, (int)(ObjectMps .size()/2), 0.01);

        cv::Point3f PoseStd2;
        cv::Point3f ObjectPose2;
        ObjectPose2 = ComputeObjectPose(vObjectMPsFilter2, PoseStd2);
        std::cout << "Box Std: " << cv::norm(PoseStd2) << std::endl;

        std::vector<MapPoint*> vObjectMPsFilter;
        cv::Point3f ObjectPose;
        double PoseStd;

        if(cv::norm(PoseStd1) == 0 || cv::norm(PoseStd2) == 0)
        {
            if(cv::norm(PoseStd1) == 0)
            {
                ObjectPose = ObjectPose2;
                vObjectMPsFilter = vObjectMPsFilter2;
                PoseStd = cv::norm(PoseStd2);
            }
            else
            {
                ObjectPose = ObjectPose1;
                vObjectMPsFilter = vObjectMPsFilter1;
                PoseStd = cv::norm(PoseStd1);
            }
        }
        else
        {
            if(cv::norm(PoseStd1) > cv::norm(PoseStd2))
            {
                ObjectPose = ObjectPose2;
                vObjectMPsFilter = vObjectMPsFilter2;
                PoseStd = cv::norm(PoseStd2);
            }
            else
            {
                ObjectPose = ObjectPose1;
                vObjectMPsFilter = vObjectMPsFilter1;
                PoseStd = cv::norm(PoseStd1);
            }
        }
        std::cout << PoseStd << std::endl;

		std::cout << "Loop closing Update object Object " << iObject << " " << ObjectMps.size() << " -> " << vObjectMPsFilter.size() <<  std::endl;
        if(vObjectMPsFilter.size() < 10 || PoseStd > 0.08)
        {
        	LCRemove.at(iObject) = true;
            //remove MapPoint outlier label;
            for(size_t j = 0; j < ObjectMps.size(); j++)
            {
                ObjectMps.at(j)->setClassId(-1);
                ObjectMps.at(j)->setObjectId(-1);
            }
            continue;
        }

		std::set<MapPoint*> sObjectMPsFilter(vObjectMPsFilter.begin(), vObjectMPsFilter.end());

		//remove MapPoint outlier label;
		for(size_t j = 0; j < ObjectMps.size(); j++)
		{
			if(sObjectMPsFilter.count(ObjectMps.at(j)))
			{
				continue;
			}
			else
			{
				ObjectMps.at(j)->setClassId(-1);
				ObjectMps.at(j)->setObjectId(-1);
			}
		}
		mvObjectPose.at(iObject) = ObjectPose;
		mvObjectBox.at(iObject) = ComputeRange(vObjectMPsFilter);
	}
}

std::vector<float> Map::ComputeRange(std::vector<MapPoint*> & Mps)
{
	std::vector<float> xv(Mps.size(), -1);
	std::vector<float> yv(Mps.size(), -1);
	std::vector<float> zv(Mps.size(), -1);
	for(size_t iMp = 0; iMp < Mps.size(); iMp++)
	{
		MapPoint* CurrentMp = Mps.at(iMp);
		cv::Mat pos = CurrentMp->GetWorldPos();
		xv.at(iMp) = pos.at<float>(0);
		yv.at(iMp) = pos.at<float>(1);
		zv.at(iMp) = pos.at<float>(2);
	}
	std::vector<float> xvs;
	std::vector<float> yvs;
	std::vector<float> zvs;
	cv::sort(xv, xvs, CV_SORT_ASCENDING);
	cv::sort(yv, yvs, CV_SORT_ASCENDING);
	cv::sort(zv, zvs, CV_SORT_ASCENDING);

	float minx = xvs.at(0);
	float maxx = xvs.at(xvs.size()-1);

	float miny = yvs.at(0);
	float maxy = yvs.at(yvs.size()-1);

	float minz = zvs.at(0);
	float maxz = zvs.at(zvs.size()-1);

	std::vector<float> out(6, 0.0);

	out.at(0) = minx;
	out.at(1) = maxx;
	out.at(2) = miny;
	out.at(3) = maxy;
	out.at(4) = minz;
	out.at(5) = maxz;

	return out;
}


std::vector<cv::Point3f> Map::GetObjectPose()
{
	unique_lock<mutex> lock(mMutexMap);
	return mvObjectPose;
}

void Map::AddObjectMapPoints(const std::vector<MapPoint*>& OMPs)
{
	unique_lock<mutex> lock(mMutexMap);
	mvvObjectMapPoints.push_back(OMPs);
}

void Map::AddObjectPose(const cv::Point3f& p, const int Id, const std::vector<float>& range)
{
	unique_lock<mutex> lock(mMutexMap);
	mvObjectPose.push_back(p);
	mvObjectId.push_back(Id);
	mvObjectBox.push_back(range);
	LCRemove.push_back(false);
//	std::cout << "Object in Map: " <<  mvObjectPose.size() <<  std::endl;
}

void Map::SetBBoxInMap(std::vector<std::vector<cv::Point3f>> & ObjectCorners, std::vector<std::vector<MapPoint*>> & vvMps)
{
    unique_lock<mutex> lock(mMutexMap);

    mObject2DBBox.resize(ObjectCorners.size());
	for(size_t i = 0; i < ObjectCorners.size(); i++)
	{
        mObject2DBBox.at(i).resize(4);
        for(size_t j = 0; j < 4; j++)
		{
            mObject2DBBox.at(i).at(j) = ObjectCorners.at(i).at(j);
		}
	}


    mvvObejctMapPointTemp.resize(vvMps.size());
	for(size_t i = 0; i < vvMps.size(); i++)
	{
        mvvObejctMapPointTemp.at(i).resize(vvMps.at(i).size());

		for(size_t j = 0; j < vvMps.at(i).size(); j++)
		{
            mvvObejctMapPointTemp.at(i).at(j) =  vvMps.at(i).at(j);
		}
	}

}

void Map::GetBBoxInMap(std::vector<std::vector<cv::Point3f>>& vvCorners, std::vector<std::vector<MapPoint*>>& vvMps)
{
    unique_lock<mutex> lock(mMutexMap);

    if(mObject2DBBox.size() == 0)
        return;

    vvCorners.resize(mObject2DBBox.size());

    for(size_t i = 0; i < mObject2DBBox.size(); i++)
	{
		vvCorners.at(i).resize(4);
        for(size_t j = 0; j < 4; j++)
		{
            vvCorners.at(i).at(j) = mObject2DBBox.at(i).at(j);
		}
	}

    std::vector<std::vector<cv::Point3f>>().swap(mObject2DBBox);

    vvMps.resize(mvvObejctMapPointTemp.size());

    for(size_t i = 0; i < mvvObejctMapPointTemp.size(); i++)
	{
        vvMps.at(i).resize(mvvObejctMapPointTemp.at(i).size());
        for(size_t j = 0; j < mObject2DBBox.size(); j++)
		{
            vvMps.at(i).at(j) = mvvObejctMapPointTemp.at(i).at(j);
		}
	}
    std::vector<std::vector<MapPoint*>>().swap(mvvObejctMapPointTemp);

//	delete [] mvvObejctMapPointTemp;

}

//void Map::SetObjectMapPoints(const std::vector<std::vector<MapPoint *> >& vvMps)
//{
//    unique_lock<mutex> lock(mMutexMap);
//    mvvObjectMapPoints = vvMps;
//}

std::vector<std::vector<float>> Map::GetObjectBoundingBox()
{
	unique_lock<mutex> lock(mMutexMap);
	return mvObjectBox;
}

std::vector<std::vector<MapPoint*>>  Map::GetObjectMapPoints()
{
	unique_lock<mutex> lock(mMutexMap);
	return mvvObjectMapPoints;
}

void Map::AddMapPoint(MapPoint *pMP)
{
	unique_lock<mutex> lock(mMutexMap);
	mspMapPoints.insert(pMP);
	//    if(pMP->mnObjectClass > -1)
	//    {
	//        mnLabeledMP++;
	//    }
}

void Map::EraseMapPoint(MapPoint *pMP)
{
	unique_lock<mutex> lock(mMutexMap);
	mspMapPoints.erase(pMP);
	//    if(pMP->mnObjectClass > -1)
	//    {
	//        mnLabeledMP--;
	//    }
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
	mvvObjectMapPoints.clear();
	//    mnLabeledMP = 0;
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

KeyFrame* Map::_ReadKeyFrame(ifstream &f, ORBVocabulary &voc, std::vector<MapPoint*> amp, ORBextractor* orb_ext)
{
	Frame fr;
	fr.SetCameraParameters(mK, mDistCoef);
	fr.mpORBvocabulary = &voc;
	f.read((char*)&fr.mnId, sizeof(fr.mnId));              // ID
	//cerr << " reading keyfrane id " << fr.mnId << endl;
	f.read((char*)&fr.mTimeStamp, sizeof(fr.mTimeStamp));  // timestamp
	cv::Mat Tcw(4,4,CV_32F);                               // position
	f.read((char*)&Tcw.at<float>(0, 3), sizeof(float));
	f.read((char*)&Tcw.at<float>(1, 3), sizeof(float));
	f.read((char*)&Tcw.at<float>(2, 3), sizeof(float));
	Tcw.at<float>(3,3) = 1.;
	cv::Mat Qcw(1,4, CV_32F);                             // orientation
	f.read((char*)&Qcw.at<float>(0, 0), sizeof(float));
	f.read((char*)&Qcw.at<float>(0, 1), sizeof(float));
	f.read((char*)&Qcw.at<float>(0, 2), sizeof(float));
	f.read((char*)&Qcw.at<float>(0, 3), sizeof(float));
	Converter::RmatOfQuat(Tcw, Qcw);
	fr.SetPose(Tcw);
	f.read((char*)&fr.N, sizeof(fr.N));                    // nb keypoints
	fr.mvKeys.reserve(fr.N);
	fr.mDescriptors.create(fr.N, 32, CV_8UC1);
	fr.mvpMapPoints = vector<MapPoint*>(fr.N,static_cast<MapPoint*>(NULL));
	for (int i=0; i<fr.N; i++) {
		cv::KeyPoint kp;
		f.read((char*)&kp.pt.x,     sizeof(kp.pt.x));
		f.read((char*)&kp.pt.y,     sizeof(kp.pt.y));
		f.read((char*)&kp.size,     sizeof(kp.size));
		f.read((char*)&kp.angle,    sizeof(kp.angle));
		f.read((char*)&kp.response, sizeof(kp.response));
		f.read((char*)&kp.octave,   sizeof(kp.octave));
		fr.mvKeys.push_back(kp);
		for (int j=0; j<32; j++)
			f.read((char*)&fr.mDescriptors.at<unsigned char>(i, j), sizeof(char));
		unsigned long int mpidx;
		f.read((char*)&mpidx,   sizeof(mpidx));
		if (mpidx == ULONG_MAX)	fr.mvpMapPoints[i] = NULL;
		else fr.mvpMapPoints[i] = amp[mpidx];
	}
	// mono only for now
	fr.mvuRight = vector<float>(fr.N,-1);
	fr.mvDepth = vector<float>(fr.N,-1);
	fr.mpORBextractorLeft = orb_ext;
	fr.InitializeScaleLevels();
	fr.UndistortKeyPoints();
	fr.AssignFeaturesToGrid();
	fr.ComputeBoW();

	KeyFrame* kf = new KeyFrame(fr, this, NULL);
	kf->mnId = fr.mnId; // bleeee why? do I store that?
	for (int i=0; i<fr.N; i++) {
		if (fr.mvpMapPoints[i]) {
			fr.mvpMapPoints[i]->AddObservation(kf, i);
			if (!fr.mvpMapPoints[i]->GetReferenceKeyFrame()) fr.mvpMapPoints[i]->SetReferenceKeyFrame(kf);
		}
	}

	return kf;
}

MapPoint* Map::_ReadMapPoint(ifstream &f) {
	long unsigned int id;
	f.read((char*)&id, sizeof(id));              // ID
	cv::Mat wp(3,1, CV_32F);
	f.read((char*)&wp.at<float>(0), sizeof(float));
	f.read((char*)&wp.at<float>(1), sizeof(float));
	f.read((char*)&wp.at<float>(2), sizeof(float));
	long int mnFirstKFid=0, mnFirstFrame=0;
	MapPoint* mp = new MapPoint(wp, mnFirstKFid, mnFirstFrame, this);
	mp->mnId = id;
	return mp;
}


bool Map::Load(const string &filename, ORBVocabulary &voc, const string &settingFile)
{
	// Load ORB parameters and camera parameters
	cv::FileStorage fSettings(settingFile, cv::FileStorage::READ);
	int nFeatures = fSettings["ORBextractor.nFeatures"];
	float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
	int nLevels = fSettings["ORBextractor.nLevels"];
	int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
	int fMinThFAST = fSettings["ORBextractor.minThFAST"];

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

	ORB_SLAM2::ORBextractor orb_ext = ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
	fSettings.release();

	cerr << "Map: reading from " << filename << endl;
	ifstream f;
	f.open(filename.c_str());

	long unsigned int nb_mappoints, max_id=0;
	f.read((char*)&nb_mappoints, sizeof(nb_mappoints));
	cerr << "reading " << nb_mappoints << " mappoints" << endl;
	for (unsigned int i=0; i<nb_mappoints; i++) {
		ORB_SLAM2::MapPoint* mp = _ReadMapPoint(f);
		if (mp->mnId>=max_id) max_id=mp->mnId;
		AddMapPoint(mp);
	}
	ORB_SLAM2::MapPoint::nNextId = max_id+1; // that is probably wrong if last mappoint is not here :(

	std::vector<MapPoint*> amp = GetAllMapPoints();
	long unsigned int nb_keyframes;
	f.read((char*)&nb_keyframes, sizeof(nb_keyframes));
	cerr << "reading " << nb_keyframes << " keyframe" << endl;
	vector<KeyFrame*> kf_by_order;
	for (unsigned int i=0; i<nb_keyframes; i++) {
		KeyFrame* kf = _ReadKeyFrame(f, voc, amp, &orb_ext);
		AddKeyFrame(kf);
		kf_by_order.push_back(kf);
	}

	// Load Spanning tree
	map<unsigned long int, KeyFrame*> kf_by_id;
	for(auto kf: mspKeyFrames)
		kf_by_id[kf->mnId] = kf;

	for(auto kf: kf_by_order) {
		unsigned long int parent_id;
		f.read((char*)&parent_id, sizeof(parent_id));          // parent id
		if (parent_id != ULONG_MAX)
			kf->ChangeParent(kf_by_id[parent_id]);
		unsigned long int nb_con;                             // number connected keyframe
		f.read((char*)&nb_con, sizeof(nb_con));
		for (unsigned long int i=0; i<nb_con; i++) {
			unsigned long int id; int weight;
			f.read((char*)&id, sizeof(id));                   // connected keyframe
			f.read((char*)&weight, sizeof(weight));           // connection weight
			kf->AddConnection(kf_by_id[id], weight);
		}
	}
	// MapPoints descriptors
	for(auto mp: amp) {
		mp->ComputeDistinctiveDescriptors();
		mp->UpdateNormalAndDepth();
	}

#if 0
	for(auto mp: mspMapPoints)
		if (!(mp->mnId%100))
			cerr << "mp " << mp->mnId << " " << mp->Observations() << " " << mp->isBad() << endl;
#endif

#if 0
	for(auto kf: kf_by_order) {
		cerr << "loaded keyframe id " << kf->mnId << " ts " << kf->mTimeStamp << " frameid " << kf->mnFrameId << " TrackReferenceForFrame " << kf->mnTrackReferenceForFrame << endl;
		cerr << " parent " << kf->GetParent() << endl;
		cerr << "children: ";
		for(auto ch: kf->GetChilds())
			cerr << " " << ch;
		cerr <<endl;
	}
#endif
	return true;
}

} //namespace ORB_SLAM
