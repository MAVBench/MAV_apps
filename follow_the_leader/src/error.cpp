#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>
#include <limits>
#include <stdint.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Drone.h"
#include "objdetect.h"
#include "track.h"
#include "pid.h"
#include "string"
#include <fstream>
#include <iostream>
#include <fstream>
#include <cmath>
#include "error.h"

using namespace std;

error::error() {
    this->x = -1;
    this->y = -1;
    this->z = -1;
    this->full = -1;
}

error::error(bounding_box bb, int img_height, int img_width, double height_ratio) {    
    double bb__cntr__x =  bb.x + bb.w/2;
    double bb__cntr__z =  bb.y + bb.h/2;
    this->x = (bb__cntr__x - img_width/2)/(img_width/2);
    this->z = (bb__cntr__z - img_height/2)/(img_height/2);
    this->y = (bb.h/img_height - height_ratio);
    this->full = pow(pow(this->x,2)+ pow(this->z,2), .5)/2; //+ pow(this->z,2),.5)/3;
}


