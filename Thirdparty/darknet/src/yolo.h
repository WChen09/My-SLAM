#ifndef DARKNET_YOLO_H
#define DARKNET_YOLO_H

#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>

#include "parser.h"
#include "utils.h"
#include "option_list.h"
#include "region_layer.h"
#include "image.h"

#include "object.h"
#include <string>
#include <vector>

using namespace std;
class Yolo {
public:

    Yolo();

    void detect(const cv::Mat& img, std::vector<DetectedObject>& detection)const; //Throws
    void readConfig(std::string filename);
    void loadConfig();
    char* getImgPath();
    string getVideoPath();

//    char** getNames();

    vector<string> get_labels_();
private:
    void ocv_to_yoloimg(const cv::Mat& img, image& yolo_img)const;

    string datacfg;
    string cfgfile;
    string weightfile;
    string alphabet_path;
    string name_list;
    string imgPathFile;
    string videoFile;

    float thresh;
    float hier_thresh;
    float nms;

    int nClass = 0;

    //members
    network net;
    char **names;
    image **alphabet;

    //Avoid copy by keeping these unimplemented
    const Yolo& operator=(const Yolo& rhs);
    Yolo(const Yolo& copy);
};

#endif //DARKNET_YOLO_H
