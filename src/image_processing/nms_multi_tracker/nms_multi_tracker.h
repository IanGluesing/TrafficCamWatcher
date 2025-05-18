#pragma once

#include "tracking_base.h"

#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>

struct custom_tracker {
    cv::Ptr<cv::legacy::Tracker> tracker_impl;
    cv::Rect2d bounding_box;
    double centroid_displacement_vs_last_frame;
    int insufficient_movement_frames_in_a_row;
    int failed_updates_in_a_row;
};

class nms_multi_tracker: 
    public base_image_tracker,
    public cv::legacy::MultiTracker {
    
    public:

        nms_multi_tracker(Video_Camera* camera_to_process);

        virtual ~nms_multi_tracker() = default;

        // Override start to take initial frame from frame stream
        void start() override;

        inline void process_msg(const zmq::message_t& identity, const zmq::message_t& payload) override;
    
    private:

        bool current_tracker_still_valid(int index_of_tracker);

        cv::Mat previous_frame;
        cv::Mat current_frame;
        cv::Mat diff;

        std::vector<custom_tracker> current_trackers;
};