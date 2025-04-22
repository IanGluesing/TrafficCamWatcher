#pragma once
#include <string>

class Camera {
    public:

        Camera(std::string url) {
            this->url = url;
        }

        std::string get_url() {
            return this->url;
        }

    private:

        std::string url;
};