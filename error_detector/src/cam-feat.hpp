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
public:
    /**
     * Constructor
     * @param[in]   frame_thresh    How many frames without information
                                    before deemed an error?
     */
    cam_feat(int frame_threh)
    {
        this->fault_rating = 0;
        this->frame_thresh = frame_thresh;
        this->orb = cv::ORB::create(500, 1.2f, 4, 5, 0, 2, cv::ORB::HARRIS_SCORE, 5);
    };

    /**
     * Provides a frame to this feature detector. Increments internal
     * counters for thresholds if necessary
     *
     * @param[in]   frame   A color frame. Best if downscaled for performance.
     * @return  true if no errors have been detected with regards to features
     */
    bool test_frame(const cv::Mat & frame)
    {
        // ORB uses greyscale
        cv::Mat grey;
        cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
        std::vector<cv::KeyPoint> kp;
        this->orb->detect(grey, kp);

        if (kp.size() < 3) {
            if (fault_rating <= INT_MAX - 1) {
                fault_rating += 1;
            }
        } else {
            fault_rating = 0;
        }

        return this->has_features();
    };

    /**
     * Tells if there are features in the tested frames.
     * @return true if okay, false if faulty
     */
    bool has_features(void)
    {
        return !(this->is_faulty());
    };

    /**
     * Tells if there are faults in the tested frames
     * @return true if faulty
     */
    bool is_faulty(void)
    {
        return (fault_rating > this->frame_thresh);
    };

private:
    int fault_rating;
    int frame_thresh;
    cv::Ptr<cv::ORB> orb;
};

#endif//__CAM_FEAT_HPP__
