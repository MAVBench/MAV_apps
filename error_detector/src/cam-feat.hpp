#ifndef __CAM_FEAT_HPP__
#define __CAM_FEAT_HPP__

#include <climits>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

/**
 * This class provides a context for detecting camera errors via image features
 * It utilizes OpenCV's ORB feature detector, since SIFT and SURF are
 * patented.
 */
class cam_feat
{
private:
    struct data_point
    {
        //std::vector<cv::KeyPoint> * kp = nullptr;
        cv::Mat * desc = nullptr;
        int count = -1;
    };

    // count of current frame
    int calc_spuriosity(int count)
    {
        int spuriosity = 0;
        for (auto it = this->data.begin(); it != this->data.end(); ++it) {
            int old_count = it->count;
            float ratio = (float) count / (float) old_count;
            if (ratio < 0.25f) {
                ++spuriosity;
            } else {
                --spuriosity;
            }
        }
        return spuriosity;
    };

public:
    enum status
    {
        OKAY,
        BLANK,
        HIJACK
    };

    /**
     * Constructor
     * @param[in]   frame_thresh    How many frames without information
     *                              before deemed an error?
     * @param[in]   hijack_detect   Enable detection of hijacked feed?
     * @param[in]   hijack_thresh   How many spurious frames in a row to consider a fault?
     * @param[in]   his_size        History size additional detection
     */
    cam_feat(int frame_threh, bool hijack_detect, int hijack_thresh = 3, int his_size = 4)
    {
        this->fault_rating = 0;
        this->frame_thresh = frame_thresh;
        this->orb = cv::ORB::create(500, 1.2f, 4, 5, 0, 2, cv::ORB::HARRIS_SCORE, 5);

        this->hijack_detect = hijack_detect;
        this->hijack_rating = 0;
        if (hijack_detect)
            this->matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        this->hijack_thresh = hijack_thresh;
        this->his_size = his_size;
    };

    /**
     * Provides a frame to this feature detector. Increments internal
     * counters for thresholds if necessary
     *
     * @param[in]   frame   A color frame. Best if downscaled for performance.
     * @return  true if no errors have been detected with regards to features
     */
    status test_frame(const cv::Mat & frame)
    {
        // ORB uses greyscale
        cv::Mat grey;
        cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);

        std::vector<cv::KeyPoint> * kp = new std::vector<cv::KeyPoint>();
        cv::Mat * desc = new cv::Mat();

        this->orb->detectAndCompute(grey, cv::noArray(), *kp, *desc);

        if (kp->size() < 3) {
            if (fault_rating <= INT_MAX - 1) {
                fault_rating += 1;
            }
        } else {
            fault_rating = 0;
        }

        // hijack detection
        if (this->hijack_detect) {
            std::vector<std::vector<cv::DMatch>> matches;

            // when we have previous frames, do a detection
            int count = 0;
            if (this->data.size() >= this->his_size) {
                if (kp->size() > 0) {
                    // count number of matched keypoints
                    for (auto it = this->data.begin(); it != this->data.end(); ++it) {
                        this->matcher->knnMatch(*(it->desc), *desc, matches, 2);
                        for (int i = 0; i < matches.size(); ++i) {
                            if (matches[i][0].distance < 0.20f * matches[i][1].distance)
                                ++count;
                        }
                    }
                    //std::cout << count << std::endl;
                }

                // thresholding
                if (this->data.front().count == -1 ||
                    calc_spuriosity(count) < (-this->hijack_rating)) {
                    data_point datum;
                    datum.desc = desc;
                    datum.count = count;
                    this->data.push_back(datum);
                    hijack_rating = 0;
                } else {
                    //std::cout << "spurious" << std::endl;
                    if (hijack_rating <= INT_MAX - 1)
                        hijack_rating += 1;
                }
            } else {
                data_point datum;
                datum.desc = desc;
                datum.count = -1;
                this->data.push_back(datum);
            }

            if (this->data.size() > this->his_size) {
                data_point datum = this->data.front();
                delete datum.desc;
                this->data.pop_front();
            }
        }

        return this->get_status();
    };

    /**
     * Tells if there are faults in the tested frames
     * @return true if faulty
     */
    status get_status(void)
    {
        if (this->fault_rating > this->frame_thresh) {
            return BLANK;
        }

        if (hijack_detect) {
            if (this->hijack_rating > this->hijack_thresh)
                return HIJACK;
        }

        return OKAY;
    };

private:
    int fault_rating;
    int frame_thresh;
    cv::Ptr<cv::ORB> orb;

    int his_size;
    int hijack_rating;
    int hijack_thresh;
    bool hijack_detect;

    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::list<data_point> data;
};

#endif//__CAM_FEAT_HPP__
