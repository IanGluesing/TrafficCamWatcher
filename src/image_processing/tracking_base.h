#pragma once

#include "data_handler.h"
#include "data_forwarder.h"
#include "camera.h"

class base_image_tracker: public data_handler {
    public:

        base_image_tracker(Video_Camera* camera_to_process);

        virtual ~base_image_tracker() = default;

        cv::Mat get_current_frame() {
            return display_frame;
        }

        std::string get_url() {
            return camera_to_process->get_url();
        }

    protected:

        void send_lat_long_pair(const double& point_lattitude, const double& point_longitude);

        data_forwarder df;
        Video_Camera* camera_to_process;
        int received_frame_count;
        cv::Mat display_frame;
};