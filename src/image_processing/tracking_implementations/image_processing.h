#pragma once

#include "tracking_base.h"

#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>

class image_processing: public base_image_tracker {
    public:

        image_processing(Video_Camera* camera_to_process);

        virtual ~image_processing() = default;

        inline void process_msg(const zmq::message_t& identity, const zmq::message_t& payload) override;
    
    private:
        
        cv::Ptr<cv::BackgroundSubtractor> background_subtractor;

        cv::Ptr<cv::legacy::MultiTracker> multi_tracker;

        bool initialized;

        cv::Mat fgMask;

        int count;
};