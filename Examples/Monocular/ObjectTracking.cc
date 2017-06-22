#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>

#include "stats.h" // Stats structure definition
#include "utils.h" // Drawing and printing functions
#include "ObjectTracker.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Thirdparty/darknet/src/yolo.h"
#include "Thirdparty/darknet/src/object.h"

using namespace std;
using namespace cv;

void LoadImages_robotcar(const string &strSequence, vector<string> &vstrImageFilenames);

int main(int argc, char **argv)
{
    if(argc < 3) {
        cerr << "Usage: " << endl <<
                "./ObjectTracking img_Path YOLO_cfg_path" << endl;
        return 1;
    }

    Yolo yolo;
    yolo.readConfig(argv[2]);

    int maxdistTh = 0.8;
    cv::Size imgSize(1280, 960);
    ObjectTracker* track = new ObjectTracker(maxdistTh, imgSize);

    std::vector<string> vImagePath;
    LoadImages_robotcar(argv[1], vImagePath);

    for(size_t iFrame = 0; iFrame != vImagePath.size(); iFrame++)
    {
        cv::Mat frame = cv::imread(vImagePath[iFrame], CV_LOAD_IMAGE_UNCHANGED);
        if (frame.empty())
        {
            break;
        }

        vObjects* currentObject = new vObjects();
        yolo.detect(frame, *currentObject);

        track->grabImgWithObjects(frame, *currentObject);

    }

    return 0;
}

void LoadImages_robotcar(const string &strPathToSequence, vector<string> &vstrImageFilenames)
{

    string strPrefixLeft = strPathToSequence + "imagePath.txt";
    ifstream pathFile(strPrefixLeft.c_str());
    if(!pathFile.is_open()){
        cerr << "Cannot open " << strPrefixLeft << endl;
        exit(0);
    }

    while(!pathFile.eof())
    {
        string s;
        getline(pathFile, s);
        if(!s.empty()){
            vstrImageFilenames.push_back(s);
        }
    }
    pathFile.close();
}
