#include "track.h"
#include <chrono>
#include <bits/stdc++.h>
#include <stdio.h>

KCFtracker::KCFtracker(const bounding_box& bb, cv::Mat frame) : tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB)
{     
    tracker.init(cv::Rect(bb.x, bb.y, bb.w, bb.h), frame);
    peak = 0;
}

bounding_box KCFtracker::track(cv::Mat frame) {
    bounding_box result;
    double old_peak = peak;
    
    cv::Rect rect = tracker.update(frame, &peak);

    result.x = rect.x;
    result.y = rect.y;
    result.w = rect.width;
    result.h = rect.height;
    result.conf = peak != 0 ? peak/old_peak : 1.0;

    return result;
}
