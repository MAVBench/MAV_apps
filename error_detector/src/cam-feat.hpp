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
    struct his_entry
    {
        //std::vector<cv::KeyPoint> * kp = nullptr;
        cv::Mat * desc = nullptr;
        int count = -1;

        void clean(void)
        {
            delete this->desc;
        }
    };

    // count of current frame
    int calc_spuriosity(int count)
    {
        int spuriosity = 0;
        for (auto it = this->his.begin(); it != this->his.end(); ++it) {
            int old_count = it->count;
            if (old_count > 300)
                old_count = 300;
            float ratio = (float) count / (float) old_count;
            if (ratio < 0.15f) {
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

    ~cam_feat(void)
    {
        this->reset_hijack();
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
        cv::equalizeHist(grey, grey);

        bool pushed = false;
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
        if (this->hijack_detect && kp->size() >= 3) {
            std::vector<std::vector<cv::DMatch>> matches;

            float mean = cv::mean(grey)[0];
            //float ratio_thresh = (mean[0] < 40.0) ? 0.30f : 0.20f;
            float ratio_thresh = 0.30f;

            //ratio_thresh = -(0.3)/(130.0) * mean + 0.44615;
            //ratio_thresh = -(0.3)/(130.0) * mean + 0.54615;
            //ratio_thresh = -(0.3)/(130.0) * mean + 0.64615;
            ratio_thresh = -(0.3)/(130.0) * mean + 0.59615;

            if (ratio_thresh > 0.40f)
                ratio_thresh = 0.40f;
            if (ratio_thresh < 0.10f)
                ratio_thresh = 0.10f;

            // when we have previous frames, do a detection
            int count = 0;
            if (this->his.size() >= this->his_size) {
                if (kp->size() > 0) {
                    // count number of matched keypoints
                    for (auto it = this->his.begin(); it != this->his.end(); ++it) {
                        this->matcher->knnMatch(*(it->desc), *desc, matches, 2);
                        for (int i = 0; i < matches.size(); ++i) {
                            //if (matches[i][0].distance < 0.30f * matches[i][1].distance)
                            if (matches[i][0].distance < ratio_thresh * matches[i][1].distance)
                                ++count;
                        }
                    }
                    //std::cout << count << ", kps: " << kp->size() << ", mean: " << mean << std::endl;
                }

                // thresholding
                if (this->his.front().count == -1 ||
                    calc_spuriosity(count) < (-this->hijack_rating)) {
                    his_entry entry;
                    entry.desc = desc;
                    entry.count = count;
                    this->his.push_back(entry);
                    hijack_rating = 0;
                    pushed = true;
                } else {
                    //std::cout << "spurious" << std::endl;
                    if (hijack_rating <= INT_MAX - 1)
                        hijack_rating += 1;
                }
            } else {
                his_entry entry;
                entry.desc = desc;
                entry.count = -1;
                this->his.push_back(entry);
                pushed = true;
            }

            if (this->his.size() > this->his_size) {
                his_entry entry = this->his.front();
                entry.clean();
                this->his.pop_front();
            }
        }

        if (!pushed) {
            delete desc;
        }
        delete kp;

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

    /**
     * Resets hijack state
     * Flushes history
     */
     void reset_hijack(void)
     {
        while (!(this->his.empty())) {
            his_entry entry = this->his.front();
            entry.clean();
            this->his.pop_front();
        }
        hijack_rating = 0;
     }


private:
    int fault_rating;
    int frame_thresh;
    cv::Ptr<cv::ORB> orb;

    int his_size;
    int hijack_rating;
    int hijack_thresh;
    bool hijack_detect;

    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::list<his_entry> his;
};

#endif//__CAM_FEAT_HPP__
