#pragma once

#include "tracking_base.h"

class non_maximum_suppression: 
    public base_image_tracker {
    
    public:

        non_maximum_suppression(Video_Camera* camera_to_process);

        virtual ~non_maximum_suppression() = default;

        // Override start to take initial frame from frame stream
        void start() override;

        inline void process_msg(const zmq::message_t& identity, const zmq::message_t& payload) override;
    
    private:

        cv::Mat previous_frame;
        cv::Mat current_frame;
        cv::Mat diff;
};