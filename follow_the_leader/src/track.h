#ifndef TRACK_H
#define TRACK_H

#include <opencv2/opencv.hpp>
#include "bounding_box.h"

#include "kcf/kcftracker.hpp"

class KCFtracker {
public:
    KCFtracker(const bounding_box& bb, cv::Mat frame);
    bounding_box track(cv::Mat frame);
private:
    KCFTracker tracker;

    const bool HOG = true;
    const bool FIXEDWINDOW = false;
    const bool MULTISCALE = true;
    const bool LAB = false;

    double peak;
};

#endif
