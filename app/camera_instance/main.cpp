
#include "camera.h"

int main() {

    std::vector<Video_Camera*> list_of_cameras_from_json;
    parse_camera_json(list_of_cameras_from_json);

    for (Video_Camera* c: list_of_cameras_from_json) {
        c->start_thread();
    }

    while (true) {
        
    }
    
    return 0;
}
