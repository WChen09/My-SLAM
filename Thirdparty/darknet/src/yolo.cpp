#include "yolo.h"
#include <stdexcept>

#include <opencv2/highgui/highgui.hpp>
#include <fstream>

#include <chrono>

Yolo::Yolo(const string &filename)
{
    cv::FileStorage file(filename, cv::FileStorage::READ);
    if (!file.isOpened()){
        std::cerr << "failed to open config file " << filename <<  std::endl;
        exit(0);
    }

    file["dataPath"] >> datacfg;

    file["netCfgPath"] >> cfgfile;

    file["weightPath"] >> weightfile;

    file["nameListPath"] >> name_list;

    file["thresh"] >> thresh;

    file["hier_thresh"] >> hier_thresh;

    file["nms"] >> nms;

    file["alphabetPath"] >> alphabet_path;

    file["imgPathFile"] >> imgPathFile;

    file["saveVideo"] >> videoFile;

    file["nClass"] >> nClass;

    file.release();
    std::cout << "--------" << std::endl;
    std::cout << "loading config information: " << std::endl;
    std::cout << "dataPath: " << datacfg << std::endl;
    std::cout << "netCfgPath: " << cfgfile << std::endl;
    std::cout << "weightPath: " << weightfile << std::endl;
    std::cout << "nameListPath" << name_list << std::endl;
    std::cout << "alphabetPath" << alphabet_path << std::endl;
    std::cout << "imgPathFile" << imgPathFile << std::endl;
    std::cout << "thresh " << thresh << std::endl;
    std::cout << "hier_thresh " << hier_thresh << std::endl;
    std::cout << "nms " << nms << std::endl;
    std::cout << "nClass " << nClass << std::endl;

    loadConfig();

}

void Yolo::ocv_to_yoloimg(const cv::Mat& img, image& yolo_img)const
{
    yolo_img = make_image(img.size().width, img.size().height, img.channels());
    unsigned char *data = img.data;
    int h = img.size().height;
    int w = img.size().width;
    int c = img.channels();
    int step = w*c;
    int i, j, k, count=0;

    for(k= 0; k < c; ++k){
        for(i = 0; i < h; ++i){
            for(j = 0; j < w; ++j){
                yolo_img.data[count++] = data[i*step + j*c + k]/255.;
            }
        }
    }
}

void Yolo::readConfig(std::string filename){

    cv::FileStorage file(filename, cv::FileStorage::READ);
    if (!file.isOpened()){
        std::cerr << "failed to open config file " << filename <<  std::endl;
        exit(0);
    }

    file["dataPath"] >> datacfg;

    file["netCfgPath"] >> cfgfile;

    file["weightPath"] >> weightfile;

    file["nameListPath"] >> name_list;

    file["thresh"] >> thresh;

    file["hier_thresh"] >> hier_thresh;

    file["nms"] >> nms;

    file["alphabetPath"] >> alphabet_path;

    file["imgPathFile"] >> imgPathFile;

    file["saveVideo"] >> videoFile;

    file["nClass"] >> nClass;

    file.release();
    std::cout << "--------" << std::endl;
    std::cout << "loading config information: " << std::endl;
    std::cout << "dataPath: " << datacfg << std::endl;
    std::cout << "netCfgPath: " << cfgfile << std::endl;
    std::cout << "weightPath: " << weightfile << std::endl;
    std::cout << "nameListPath" << name_list << std::endl;
    std::cout << "alphabetPath" << alphabet_path << std::endl;
    std::cout << "imgPathFile" << imgPathFile << std::endl;
    std::cout << "thresh " << thresh << std::endl;
    std::cout << "hier_thresh " << hier_thresh << std::endl;
    std::cout << "nms " << nms << std::endl;
    std::cout << "nClass " << nClass << std::endl;

    //    //initialization
    //    names = (char**)std::malloc(nClass*sizeof(char *));
    //    for(int i = 0; i < nClass; i++){
    //        names[i] = (char*)std::malloc(15);
    //    }
}

void Yolo::loadConfig(){

    //setConfigFile
    net = parse_network_cfg(const_cast<char*>(cfgfile.c_str()));

    //setWeightFile
    if(!net.n)
    {
        throw std::runtime_error("Can't load weights, there is a problem with network generation");
    }

    load_weights(&net, const_cast<char*>(weightfile.c_str()));
    set_batch_network(&net, 1);

    //setAlphabet
    alphabet = load_alphabet_custom(const_cast<char*>(alphabet_path.c_str()));
}

char* Yolo::getImgPath(){
    return const_cast<char*>(imgPathFile.c_str());
}

string Yolo::getVideoPath(){
    return videoFile;
}

vector<string> Yolo::get_labels_(){

    ifstream labelFile;
    labelFile.open(name_list);
    if(!labelFile.is_open()) std::cout << "cannot open names file" << std::endl;
    vector<string> vnames;
    vnames.reserve(nClass);
    string str;
    while(getline(labelFile, str)){
        vnames.push_back(str);
    }
    labelFile.close();
    return vnames;
}

//char** Yolo::getNames()
//{
//    return names;
//}

void Yolo::detect(const cv::Mat& img, std::vector<DetectedObject>& detection)const
{
    image im, sized;
    float **probs = NULL;
    cv::Mat img_local;

    try {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        if(img.channels() == 1){
            cv::cvtColor(img,img_local,CV_GRAY2RGB);
        }
        else{
            cv::cvtColor(img, img_local, cv::COLOR_BGR2RGB);
        }
        ocv_to_yoloimg(img_local, im);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        sized = resize_image(im, net.w, net.h);
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

        layer l = net.layers[net.n - 1];
        int output_size = l.w * l.h * l.n;

        box boxes[output_size];
        probs = new float *[output_size]();
        for (int i = 0; i < output_size; i++)
            probs[i] = new float[l.classes + 1]();

        network_predict(net, sized.data);
        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();

        std::cout << "  reshape duration: " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() << std::endl
                  << "  resize duration: " << std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count() << std::endl
                  << "  prediction duration: " << std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count()
                  << std::endl;
        get_region_boxes(l, 1, 1, thresh, probs, boxes, 0, 0, hier_thresh);

        if (l.softmax_tree && nms)
            do_nms_obj(boxes, probs, output_size, l.classes, nms);
        else if (nms)
            do_nms_sort(boxes, probs, output_size, l.classes, nms);

        //draw_detections(im, output_size, thresh, boxes, probs, names, alphabet, 20);
        //show_image(im, "predictions");
        //cv::waitKey(1);

        for (int i = 0; i < output_size; i++) {
            int most_probable_class_index = max_index(probs[i], l.classes);
            float prob = probs[i][most_probable_class_index];
            if (prob > thresh) {
                box &b = boxes[i];
                int left = (b.x - b.w / 2.) * im.w;
                int right = (b.x + b.w / 2.) * im.w;
                int top = (b.y - b.h / 2.) * im.h;
                int bot = (b.y + b.h / 2.) * im.h;
                if (left < 0) left = 0;
                if (right > im.w - 1) right = im.w - 1;
                if (top < 0) top = 0;
                if (bot > im.h - 1) bot = im.h - 1;

                cv::Rect r(left, top, std::fabs(left - right), std::fabs(top - bot));
                detection.push_back(DetectedObject(most_probable_class_index, prob * 100., r));
            }
        }
        delete[] probs;
        free_image(im);
        free_image(sized);
    }
    catch(...)
    {
        if(probs)
            delete[] probs;

        free_image(im);
        free_image(sized);
        throw std::runtime_error("Yolo related error");
    }
}
